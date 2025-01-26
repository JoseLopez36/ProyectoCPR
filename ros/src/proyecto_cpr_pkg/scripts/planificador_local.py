#!/usr/bin/env python3

import math

import rospy
import tf
import geometry_msgs.msg as geo
import std_msgs.msg as std
import nav_msgs.msg as nav
import airsim_ros_pkgs.msg as air
import sensor_msgs.msg as sens
import proyecto_cpr_pkg.msg as cpr
import sensor_msgs.point_cloud2 as pc2

# Parámetros globales
DIST_MIN = 1.5

class PlanificadorLocal:
    def __init__(self):
        # Variables miembro
        self.pose_actual = None             # (geometry_msgs/PoseWithCovariance)
        self.trayectoria_global = None      # (nav_msgs/Path)
        self.datos_lidar = sens.PointCloud2
        self.lidar_points = []

        self.tf_listener = tf.TransformListener()

        # Suscriptores
        rospy.Subscriber('/airsim_node/car_1/car_state', air.CarState, self.state_callback)
        rospy.Subscriber('/trayectoria_global', nav.Path, self.global_path_callback)
        rospy.Subscriber('/airsim_node/car_1/lidar/LidarCustom', sens.PointCloud2, self.lidar_callback)

        # Publicadores
        self.pub_trayectoria_local = rospy.Publisher(
            '/trayectoria_local', 
            nav.Path,
            queue_size=5
        )

        rospy.loginfo("Nodo de Planificador Local inicializado.")

    def state_callback(self, msg):
        # Extraer la información del estado del coche
        self.pose_actual = msg.pose             # Pose actual del coche

    def global_path_callback(self, msg):
        # Extraer la información de la trayectoria global
        self.trayectoria_global = msg           # Trayectoria global recibida

    def lidar_callback(self, msg):
        self.datos_lidar = msg

        # Convertir el mensaje PointCloud2 a una lista de puntos (x, y) ignorando z
        aux = pc2.read_points(self.datos_lidar, field_names=("x", "y"), skip_nans=True)
        self.lidar_points = [(point[0], point[1]) for point in aux]  # Obtener solo x e y
        rospy.logwarn("Lidar callbak")

    def obstaculo_detectado(self, waypoint):
        wx = waypoint.pose.position.x
        wy = waypoint.pose.position.y
        rospy.logwarn("Obstaculo detectado")

        for coord_obs in self.lidar_points:
            dist = math.sqrt((wx - coord_obs[0])**2 + (wy - coord_obs[1])**2)

            if dist <= DIST_MIN:
                return True
            
        return False

    def ejecutar_planificador_local(self):
        # Comprobar que exista trayectoria global
        if not self.trayectoria_global or len(self.trayectoria_global.poses) == 0:
            rospy.logwarn("PlanificadorLocal: No hay path global disponible. Esperando a que se reciba un camino...")
            return
        
        # Verificar si el estado del vehículo es válido
        if not self.pose_actual:
            rospy.logwarn("ControladorPurePursuit: Estado del vehiculo no valido.")
            return
        elif self.pose_actual:
            # Extraer la pose del vehículo
            try:
                # Transformar al marco de la trayectoria
                self.pose_actual.header.stamp = rospy.Time(0) # Se fuerza a obtener la última transformación disponible
                transformed_pose = self.tf_listener.transformPose(self.path.header.frame_id, self.pose_actual)
                # Extraer posición del vehículo en el marco transformado
                vehicle_pos = transformed_pose.pose.position
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("ControladorPurePursuit: Error al transformar la pose del vehículo: %s", e)
                return

        # Hallar el índice del punto más cercano en la trayectoria
        min_dist_sq = float('inf')
        closest_idx = 0
        for i, pose_stamped in enumerate(self.trayectoria_global.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist_sq = (px - vehicle_pos.x) ** 2 + (py - vehicle_pos.y) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i

        waypoints = []  # Inicializar lista para almacenar puntos transformados
        trayectoria_ajustada = nav.Path()
        trayectoria_ajustada.header = self.trayectoria_global.header
        trayectoria_ajustada.header.stamp = rospy.Time.now()

        for i, punto in enumerate(self.trayectoria_global.poses):
            # Transformar el goal a lidar_frame
            rospy.logwarn("Transformando a Lidar")
            try:
                punto.header.stamp = rospy.Time(0)
                transformed_punto = self.tf_listener.transformPose(self.datos_lidar.header.frame_id, punto)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Error de TF al transformar el objetivo a lidar_frame.")
                return False
            
            # Agregar el punto transformado a la lista
            waypoints.append(transformed_punto)

        for i in range(closest_idx,len(self.trayectoria_global.poses)):
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
                    new_waypoint = self.tf_listener.transformPose(trayectoria_ajustada.header.frame_id, waypoint)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Error de TF al transformar el objetivo a world_enu.")
                    return False
                
                trayectoria_ajustada.poses.append(new_waypoint)

            else:
                trayectoria_ajustada.poses.append(waypoint)

        # Publicar la trayectoria
        self.publish(trayectoria_ajustada)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_trayectoria_local.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(5)  # 5 Hz
        while not rospy.is_shutdown():
            self.ejecutar_planificador_local()
            rate.sleep()

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('planificador_local')

    # Crear objeto
    nodo = PlanificadorLocal()

    # Correr el nodo
    nodo.run()