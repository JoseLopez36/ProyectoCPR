#!/usr/bin/env python3

import math
import rospy
import tf
import numpy as np
import geometry_msgs.msg as geo
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import sensor_msgs.point_cloud2 as pc2

# Parámetros globales
DIST_MIN = 3

class BurbujaAlgoritmo:
    def __init__(self):
        self.traye = 0 
        self.trayectoria_global = nav.Path()
        self.trayectoria_ajustada = nav.Path()
        self.datos_lidar = sens.PointCloud2
        self.lidar_points = []

        self.tf_listener = tf.TransformListener()

        #self.local_pub = rospy.Publisher('/trayectoria_local', nav.Path, queue_size=10)

        self.global_sub = rospy.Subscriber('/trayectoria_global', nav.Path, self.copiarTray_callback)
        self.lidar_sub = rospy.Subscriber('/airsim_node/car_1/lidar/LidarCustom', sens.PointCloud2, self.lidar_callback)


    def copiarTray_callback(self, msg):
        if self.traye==1:
            return
        self.trayectoria_global = msg

        # Guardar la trayectoria recibida en la que vamos a modificar
        self.trayectoria_ajustada = nav.Path()
        self.trayectoria_ajustada.header.frame_id = self.trayectoria_global.header.frame_id
        self.trayectoria_ajustada.poses = self.trayectoria_global.poses
        self.traye=1
        rospy.logwarn("Trayectoria callbak")


    def lidar_callback(self, msg):
        self.datos_lidar = msg

        if not msg:
            rospy.logwarn("Mensaje PointCloud2 vacío.")
            return
        # Convertir el mensaje PointCloud2 a una lista de puntos (x, y) ignorando z
        aux = pc2.read_points(self.datos_lidar, field_names=("x", "y"), skip_nans=True)
        self.lidar_points = [(point[0], point[1]) for point in aux]  # Obtener solo x e y
        rospy.logwarn("Lidar callbak")


    def obstaculo_detectado(self, waypoint):
        wx = waypoint.pose.position.x
        wy = waypoint.pose.position.y
        

        for coord_obs in self.lidar_points:
            dist = math.sqrt((wx - coord_obs[0])**2 + (wy - coord_obs[1])**2)
            

            if dist <= DIST_MIN:
                rospy.logwarn("Distnacia:%d ", dist)

                return True
            
        return False


    def ajustar_trayectoria(self):
        # Hallar el índice del punto más cercano en la trayectoria
        min_dist_sq = float('inf')
        closest_idx = 0

        waypoints = []  # Inicializar lista para almacenar puntos transformados

        for i, punto in enumerate(self.trayectoria_ajustada.poses):
            # Transformar el goal a lidar_frame

            rospy.logwarn("Transformando a Lidar")
            try:
                punto.header.stamp = rospy.Time(0)
                if not self.datos_lidar or not hasattr(self.datos_lidar, 'header') or not hasattr(self.datos_lidar.header, 'frame_id'):
                    rospy.logwarn("Datos LIDAR no válidos o frame_id no encontrado.")
                    return

                transformed_punto = self.tf_listener.transformPose(self.datos_lidar.header.frame_id, punto)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Error de TF al transformar el objetivo a lidar_frame.")
                return False
            
            # Agregar el punto transformado a la lista
            waypoints.append(transformed_punto)


        for i, punto in enumerate(waypoints):
            
            px = punto.pose.position.x
            py = punto.pose.position.y

            dist_sq = (px) ** 2 + (py) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i

        for i in range(closest_idx,len(self.trayectoria_ajustada.poses)):
            waypoint = waypoints[i]
            
            if self.obstaculo_detectado(waypoint):
                rospy.loginfo("Ajustando waypoint %d: colisión detectada.", i)

                # Vector waypoint en frame LIDAR
                wx = waypoint.pose.position.x
                wy = waypoint.pose.position.y

                # Magnitud del vector
                mag = math.sqrt(wx**2 + wy**2)
                if mag < 1e-6:
                    continue  # Si el waypoint está muy cerca del LIDAR, saltamos

                # Vectores normalizados perpendiculares (rotamos 90° el vector [wx, wy])
                perp1_x = -wy / mag
                perp1_y = wx / mag
                perp2_x = wy / mag
                perp2_y = -wx / mag

                # Probar movimientos iterativos en ambas direcciones
                max_attempts = 10
                offset_scale = DIST_MIN

                for attempt in range(max_attempts):
                    # Alternar entre direcciones
                    if attempt % 2 == 0:
                        offset_x = perp1_x * offset_scale * (attempt // 2 + 1)
                        offset_y = perp1_y * offset_scale * (attempt // 2 + 1)
                    else:
                        offset_x = perp2_x * offset_scale * (attempt // 2 + 1)
                        offset_y = perp2_y * offset_scale * (attempt // 2 + 1)

                    # Calcular nueva posición del waypoint
                    new_wx = wx + offset_x
                    new_wy = wy + offset_y
                    waypoint.pose.position.x = new_wx
                    waypoint.pose.position.y = new_wy

                    # Verificar si la nueva posición es segura
                    if not self.obstaculo_detectado(waypoint):
                        rospy.loginfo("Waypoint ajustado a posición segura: x=%.2f, y=%.2f", new_wx, new_wy)
                        break

                try:
                    waypoint.header.stamp = rospy.Time(0)
                    new_waypoint = self.tf_listener.transformPose(self.trayectoria_ajustada.header.frame_id, waypoint)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Error de TF al transformar el objetivo a world_enu.")
                    return False
                
                self.trayectoria_ajustada.poses[i] = new_waypoint

        #self.local_pub.publish(self.trayectoria_ajustada)


    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(100)  # 10 Hz
        while not rospy.is_shutdown():
            self.ajustar_trayectoria()
            rate.sleep()
        

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('algoritmo_burbuja')

    # Crear objeto
    nodo = BurbujaAlgoritmo()

    # Correr el nodo
    nodo.run()