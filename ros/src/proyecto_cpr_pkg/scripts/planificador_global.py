#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import nav_msgs.msg as nav
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class PlanificadorGlobal:
    def __init__(self):
        # Variables miembro
        self.destino = None         # (geometry_msgs/PoseStamped)
        self.mapa_global = None     # (nav_msgs/OccupancyGrid)

        # Parámetros
        

        # Suscriptores
        rospy.Subscriber('/destino', geo.PoseStamped, self.target_callback)
        rospy.Subscriber('/mapa_global', nav.OccupancyGrid, self.global_map_callback)

        # Publicadores
        self.pub_trayectoria_global = rospy.Publisher(
            '/trayectoria_global', 
            nav.Path,
            queue_size=10
        )

        rospy.loginfo("Nodo de Planificador Global inicializado.")

    def target_callback(self, msg):
        # Extraer la información del destino
        self.destino = msg                      # Destino del vehículo

    def global_map_callback(self, msg):
        # Extraer la información del mapa global
        self.mapa_global = msg                  # Mapa global recibido

    def ejecutar_planificador_global(self):
        # TODO: Implementación del planificador global


        # Construir la trayectoria global
        trayectoria = nav.Path()
        trayectoria.header.frame_id = "map"
        
        now = rospy.Time.now()
        trayectoria.header.stamp = now

        # Publicar la trayectoria
        self.publish(trayectoria)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_trayectoria_global.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.ejecutar_planificador_global()
            rate.sleep()

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('planificador_global')

    # Crear objeto
    nodo = PlanificadorGlobal()

    # Correr el nodo
    nodo.run()