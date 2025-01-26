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

    def ejecutar_planificador_local(self):
        # Comprobar que exista trayectoria global
        if not self.trayectoria_global or len(self.trayectoria_global.poses) == 0:
            rospy.logwarn("PlanificadorLocal: No hay path global disponible. Esperando a que se reciba un camino...")
            return

        # Pasar trayectoria global sin modificar
        trayectoria = self.trayectoria_global

        # Publicar la trayectoria
        self.publish(trayectoria)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_trayectoria_local.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(1)  # 1 Hz
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