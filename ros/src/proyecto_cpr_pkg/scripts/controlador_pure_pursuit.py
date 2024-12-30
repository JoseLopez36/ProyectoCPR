#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import nav_msgs.msg as nav
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class ControladorPurePursuit:
    def __init__(self):
        # Variables miembro
        self.pose_actual = None # (geometry_msgs/PoseWithCovariance)
        self.trayectoria = None # (nav_msgs/Path)

        # Parámetros
        self.lookahead_distance = 0.0

        # Suscriptores
        rospy.Subscriber('/airsim_node/car_1/car_state', air.CarState, self.state_callback)
        rospy.Subscriber('/trayectoria_local', nav.Path, self.path_callback)

        # Publicadores
        self.pub_actuadores = rospy.Publisher(
            '/cmd_actuadores', 
            cpr.CmdActuadores,
            queue_size=10
        )

        rospy.loginfo("Controlador Pure Pursuit inicializado.")

    def state_callback(self, msg):
        # Extraer la información del estado del coche
        self.pose_actual = msg.pose             # Pose actual del coche

    def path_callback(self, msg):
        # Extraer la información de la trayectoria recibida
        self.trayectoria = msg                  # Trayectoria local a seguir

    def ejecutar_pure_pursuit(self):
        # TODO: Implementación de control Pure Pursuit


        # Construir los comandos para el controlador de bajo nivel
        cmd = cpr.CmdActuadores()
        cmd.vel_lineal = 0.0
        cmd.vel_angular = 0.0
        
        now = rospy.Time.now()
        cmd.header.stamp = now

        # Publicar los comandos 
        self.publish(cmd)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_actuadores.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.ejecutar_pure_pursuit()
            rate.sleep()

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('controlador_pure_pursuit')

    # Crear objeto
    nodo = ControladorPurePursuit()

    # Correr el nodo
    nodo.run()