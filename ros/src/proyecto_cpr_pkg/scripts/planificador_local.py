#!/usr/bin/env python3

import math

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import nav_msgs.msg as nav
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class PlanificadorLocal:
    def __init__(self):
        # Variables miembro
        self.pose_actual = None             # (geometry_msgs/PoseWithCovariance)
        self.trayectoria_global = None      # (nav_msgs/Path)
        self.mapa_local = None              # (nav_msgs/OccupancyGrid)

        # Parámetros
        

        # Suscriptores
        rospy.Subscriber('/airsim_node/car_1/car_state', air.CarState, self.state_callback)
        rospy.Subscriber('/trayectoria_global', nav.Path, self.global_path_callback)
        rospy.Subscriber('/mapa_local', nav.OccupancyGrid, self.local_map_callback)

        # Publicadores
        self.pub_trayectoria_local = rospy.Publisher(
            '/trayectoria_local', 
            nav.Path,
            queue_size=10
        )

        rospy.loginfo("Nodo de Planificador Local inicializado.")

    def state_callback(self, msg):
        # Extraer la información del estado del coche
        self.pose_actual = msg.pose             # Pose actual del coche

    def global_path_callback(self, msg):
        # Extraer la información de la trayectoria global
        self.trayectoria_global = msg           # Trayectoria global recibida

    def local_map_callback(self, msg):
        # Extraer la información del mapa local
        self.mapa_local = msg                   # Mapa local recibido
        
    def generate_line(self, x1, y1, x2, y2, num_points, frame_id="world_enu"):
        """
        Genera num_points sobre la línea recta desde (x1,y1) hasta (x2,y2).
        """
        points = []
        for i in range(num_points+1):
            t = i / float(num_points)  # factor en [0, 1]
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            pose = geo.PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # Orientación trivial (identidad); se puede ajustar si quieres
            pose.pose.orientation.w = 1.0
            points.append(pose)
        return points

    def generate_arc(self, cx, cy, radius, start_angle, end_angle, num_points, frame_id="world_enu"):
        """
        Genera un arco circular de radio 'radius' centrado en (cx, cy),
        yendo desde 'start_angle' hasta 'end_angle' (en radianes).
        Ej: 90° => start_angle=0, end_angle=pi/2 (o viceversa).
        """
        points = []
        for i in range(num_points+1):
            t = i / float(num_points)
            # Ángulo interpolado
            theta = start_angle + t * (end_angle - start_angle)
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)

            pose = geo.PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            points.append(pose)
        return points

    def ejecutar_planificador_local(self):
        # Parámetros del "circuito"
        FRAME_ID = "world_enu"
        L = 100.0        # Lado del rectángulo
        R = 10.0         # Radio de la esquina
        N_LINE = 80      # Puntos para cada tramo de línea
        N_ARC = 50       # Puntos para cada arco (90°)

        # 1. Creamos la estructura de Path
        trayectoria = nav.Path()
        trayectoria.header.frame_id = FRAME_ID
        trayectoria.header.stamp = rospy.Time.now()

        # Lista final de todos los PoseStamped
        points = []

        # ---- LÍNEA 1:
        line1 = self.generate_line(x1=0, y1=0, x2=0, y2=L-R, 
                            num_points=N_LINE, frame_id=FRAME_ID)
        points += line1

        # ---- ARCO 1:
        arc1 = self.generate_arc(cx=R, cy=L-R, radius=R, 
                            start_angle=math.pi, end_angle=math.pi/2,
                            num_points=N_ARC, frame_id=FRAME_ID)
        points += arc1

        # ---- LÍNEA 2:
        line2 = self.generate_line(x1=R, y1=L, x2=L-R, y2=L,
                            num_points=N_LINE, frame_id=FRAME_ID)
        points += line2

        # ---- ARCO 2:
        arc2 = self.generate_arc(cx=L, cy=L-R, radius=R,
                            start_angle=math.pi/2, end_angle=0,
                            num_points=N_ARC, frame_id=FRAME_ID)
        points += arc2

        # ---- LÍNEA 3:
        line3 = self.generate_line(x1=L+R, y1=L-R, x2=L+R, y2=R,
                            num_points=N_LINE, frame_id=FRAME_ID)
        points += line3

        # Añadir los puntos calculados
        trayectoria.poses = points

        # Publicar la trayectoria
        self.publish(trayectoria)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_trayectoria_local.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(10)  # 10 Hz
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