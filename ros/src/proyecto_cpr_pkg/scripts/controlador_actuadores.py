#!/usr/bin/env python3

import math

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class ControladorActuadores:
    def __init__(self):
         # Inicializar el nodo
        rospy.init_node('controlador_actuadores')

        # Suscriptores
        rospy.Subscriber('/cmd_actuadores', cpr.CmdActuadores, self.cmd_actuadores_callback)       
        rospy.Subscriber('/airsim_node/car_1/car_state', air.CarState, self.state_callback)

        # Publicadores
        self.pub_control = rospy.Publisher(
            '/airsim_node/car_1/car_cmd', 
            air.CarControls,
            queue_size=10
        )

        # Variables miembro
        self.vel_lineal_deseada = 0.0    # (std_msgs/float32)
        self.steering_deseado = 0.0      # (std_msgs/float32)
        self.vel_lineal_actual = None    # (std_msgs/float32)
       #self.rpm_actual = 0.0

        #Aceleraciones máxima y mínima
        self.maxacc = 1.0
        self.minacc = -1.0

        # Parámetros
        self.Kp = 3.0
        self.Ti = 1.0
        self.Td = 0.1     
        self.Tm = 0.05           # s . Tiempo de muestro de la llamada al controlador. No debe ser menor que la actualización del estado
        
        #variables estaticas del controlador pid
        self.ek_2 = 0.0
        self.ek_1 = 0.0
        self.uk_1 = 0.0   

        self.previous_velocity = 0.0
        self.previous_time = 0.0


        #Llamada a funcion controlador de manera periodica al controlador. Con periodo de muestro independiente de la frecuencia del sensor
        #self.timer = rospy.Timer(rospy.Duration(self.Tm),self.control_callback,oneshot=False)   
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Controlador de actuadores inicializado.")

    
    def run(self):
       rospy.spin()

    def shutdown(self):                     # Apagar controlador
        rospy.loginfo("Apagando controlador de actuadores")
        #self.timer.shutdown()  
        rospy.sleep(self.Tm)
        # Publicar ultimo mensaje con acceleración nula
        self.pub_control.publish(air.CarControls())
        rospy.loginfo("Information pass to the vehicle:\n Throttle = 0.0  \n Brake = 0.0 ")
        rospy.sleep(1)

    def publish(self, datos):               # Publicar los datos computados en el nodo
        self.pub_control.publish(datos)

    def cmd_actuadores_callback(self, msg : cpr.CmdActuadores):
        # Extraer la información de velocidad y dirección
        self.vel_lineal_deseada = msg.vel_lineal      # Magnitud de la velocidad lineal deseada
        self.steering_deseado = msg.steering          #Magnitud ángulo de giro de las ruedas deseado
        

    def state_callback(self, state_msg : air.CarState):
        
        
        # Extraer la información del estado del coche
        self.vel_lineal_actual = state_msg.speed              # Magnitud de la velocidad actual del coche
        self.rpm_actual = state_msg.rpm 

        self.control_callback() 

    def control_callback(self):
    #def control_callback(self,timerEvent):
        if self.vel_lineal_actual is not None:
            rospy.loginfo(f"Car's current speed is: {self.vel_lineal_actual} m/s") 
            rospy.loginfo(f"Car's rpm are: {self.rpm_actual}") 

            #Prueba- medicion acceleracion real conseguida
            current_time = rospy.Time.now().to_sec()  
            dt = current_time - self.previous_time
            if dt > 0:
                acc_real = (self.vel_lineal_actual -  self.previous_velocity)/dt
                rospy.loginfo(f"Car's true acceleration is: {acc_real} m/s") 
            self.previous_velocity = self.vel_lineal_actual
            self.previous_time = current_time
            
            #Calculo de señal de control
            acceleration = self.pid(self.vel_lineal_deseada)

            # Contrucción los mensaje de control de los actuadores
            control_msg = air.CarControls() 
            now = rospy.Time.now()
            control_msg.header.stamp = now          
            control_msg.manual = False
            control_msg.manual_gear = 0
            control_msg.handbrake = False
            control_msg.gear_immediate = True

            control_msg.steering = -self.steering_deseado / (math.pi/4)

            if acceleration >= 0:
                control_msg.throttle = acceleration
                control_msg.brake = 0.0
            else:
                control_msg.brake = - acceleration         #brake necesita ser positivo
                control_msg.throttle = 0.0

            rospy.loginfo(f"""Información enviada a los actuadores:
                            Throttle = {control_msg.throttle} 
                            Brake = {control_msg.brake} 
                            Steering = {control_msg.steering}""")
        
            # Publicar los comandos
            self.publish(control_msg)

    def pid(self,vd:float)->float:               #Implementacón de un controlador PID
        #Aproximación de Euler II incremental
        q0=self.Kp*(1+(self.Tm/self.Ti)+(self.Td/self.Tm))
        q1=self.Kp*(-1-2*(self.Td/self.Tm))
        q2=self.Kp*self.Td/self.Tm

        #Calculo del error
        ek = vd - self.vel_lineal_actual
        rospy.loginfo(f"Error : {ek}")                #Información para debugging

        #Ley de control
        uk = self.uk_1 + q0*ek + q1*self.ek_1 + q2*self.ek_2
        a = uk

        #Saturación de la señal de control - aceleracion
        if a > self.maxacc:
            a = self.maxacc
        elif a < self.minacc:
            a = self.minacc


        #Actualizacion de variables
        self.ek_2 = self.ek_1
        self.ek_1 = ek
        self.uk_1 = a 

        return a

if __name__ == '__main__':
    try:
        # Crear instancia del nodo
        controller = ControladorActuadores()
        # Correr el nodo
        controller.run()
    except rospy.ROSInterruptException:
        pass 