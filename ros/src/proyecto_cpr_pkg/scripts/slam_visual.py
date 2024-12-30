#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import sensor_msgs.msg as sensor
import nav_msgs.msg as nav
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class SlamVisual:
    def __init__(self):
        # Variables miembro
        self.imagen = None      # (sensor_msgs/Image)
        self.K = None           # (std_msgs/float64[9])

        # Parámetros
        

        # Suscriptores
        rospy.Subscriber('/airsim_node/car_1/front_center_custom/Scene', sensor.Image, self.image_callback)
        rospy.Subscriber('/airsim_node/car_1/front_center_custom/Scene', sensor.CameraInfo, self.cam_params_callback)

        # Publicadores
        self.pub_mapa_local = rospy.Publisher(
            '/mapa_local', 
            nav.OccupancyGrid,
            queue_size=10
        )

        rospy.loginfo("Nodo de SLAM Visual inicializado.")

    def image_callback(self, msg):
        # Extraer la información de la imagen
        self.imagen = msg                   # Imagen actual de la cámara

    def cam_params_callback(self, msg):
        # Extraer los parámetros de la cámara
        self.K = msg.K                      # Matriz de parámetros intrínsecos

    def ejecutar_slam_visual(self):
        # TODO: Implementación de SLAM visual:


        # Construir el mapa local
        mapa  = nav.OccupancyGrid()
        mapa.header.frame_id = "map"
        mapa.info.resolution = 0.1  # Tamaño de celda en metros
        mapa.info.width = 100
        mapa.info.height = 100
        mapa.info.origin.position.x = 0.0
        mapa.info.origin.position.y = 0.0
        mapa.info.origin.position.z = 0.0
        mapa.info.origin.orientation.w = 1.0
        mapa.data = [0] * (mapa.info.width * mapa.info.height)  # Datos del mapa (vacío)

        now = rospy.Time.now()
        mapa.header.stamp = now

        # Publicar el mapa 
        self.publish(mapa)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_mapa_local.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.ejecutar_slam_visual()
            rate.sleep()

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('slam_visual')

    # Crear objeto
    nodo = SlamVisual()

    # Correr el nodo
    nodo.run()