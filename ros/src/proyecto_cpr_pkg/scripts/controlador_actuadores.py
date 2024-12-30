#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class ControladorActuadores:
    def __init__(self):
        # Variables miembro
        self.vel_lineal_deseada = 0.0   # (std_msgs/float32)
        self.vel_angular_deseada = 0.0  # (std_msgs/float32)
        self.vel_lineal_actual = 0.0    # (std_msgs/float32)
        self.twist = None               # (geometry_msgs/Twist)

        # Parámetros
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        # Suscriptores
        rospy.Subscriber('/cmd_actuadores', cpr.CmdActuadores, self.cmd_actuadores_callback)
        rospy.Subscriber('/airsim_node/car_1/car_state', air.CarState, self.state_callback)

        # Publicadores
        self.pub_control = rospy.Publisher(
            '/airsim_node/car_1/car_cmd', 
            air.CarControls,
            queue_size=0
        )

        rospy.loginfo("Controlador de actuadores inicializado.")

    def cmd_actuadores_callback(self, msg):
        # Extraer la información de velocidad y dirección
        self.vel_lineal_deseada = msg.vel_lineal      # Magnitud de la velocidad lineal deseada
        self.vel_angular_deseada = msg.vel_angular    # Magnitud de la velocidad angular deseada

    def state_callback(self, msg):
        # Extraer la información del estado del coche
        self.vel_lineal_actual = msg.speed              # Magnitud de la velocidad actual del coche
        self.twist = msg.twist                          # Twist actual del coche

    def ejecutar_controlador(self):
        # TODO: Implementación de control de bajo nivel


        # Construir los comandos para actuadores
        controles = air.CarControls()
        controles.steering = 0.0
        controles.throttle = 0.0
        controles.brake = 0.0
        controles.manual = False
        controles.manual_gear = 0
        controles.handbrake = False
        controles.gear_immediate = True
        
        now = rospy.Time.now()
        controles.header.stamp = now

        # Publicar los comandos
        self.publish(controles)

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_control.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.ejecutar_controlador()
            rate.sleep()

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('controlador_actuadores')

    # Crear objeto
    nodo = ControladorActuadores()

    # Correr el nodo
    nodo.run()