#!/usr/bin/env python3

import math

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr
import time

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
        #-----------debuging----------
        self.pub_debug = rospy.Publisher(
            '/debuging', 
            cpr.debuging,
            queue_size=10
        )
        #debuging-------------------

        # Variables miembro
        self.vel_lineal_deseada = 0.0    # (std_msgs/float32)
        self.steering_deseado = 0.0      # (std_msgs/float32)
        self.vel_lineal_actual = None    # (std_msgs/float32)
        
        #filtrado
        self.filtrar_vel = True  
        self.filtrar_acc = True

        self.vel_act_filtrada = 0.0
        self.vel_des_filtrada = 0.0
        self.acc_filtrada = 0.0

        #Aceleraciones máxima y mínima
        self.maxacc = 1.0
        self.minacc = -1.0

        # Parámetros
        self.Kp = 2.0                  #2.0               
        self.Ti = 1.0                  #1.0
        self.Td = 0.01                 #0.01
        self.Tm = 0.05              # s . Tiempo de muestro de la llamada al controlador. No debe ser menor que la actualización del estado
        self.T_real = 0.0           # s . Medición del tiempo entre callback del topic estado

        #variables estaticas del controlador pid
        self.ek_2 = 0.0
        self.ek_1 = 0.0
        self.uk_1 = 0.0   

        self.previous_velocity = 0.0
        self.previous_time = 0.0
        self.now = time.perf_counter()          #Guardar espacio al timer

        #Llamada a funcion controlador de manera periodica al controlador.
        #self.timer = rospy.Timer(rospy.Duration(self.Tm),self.control_callback,oneshot=False)   
        
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Controlador de actuadores inicializado.")

    
    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            rospy.logerr(f"Error inesperado: {e}") 
       

    def shutdown(self):                     # Apagar controlador
        rospy.loginfo("Apagando controlador de actuadores")
        #self.timer.shutdown()  
        rospy.sleep(0.05)
        # Publicar ultimo mensaje con acceleración nula
        self.pub_control.publish(air.CarControls())
        rospy.loginfo("Information pass to the vehicle:\n Throttle = 0.0  \n Brake = 0.0 ")
        rospy.sleep(1)

    def publish(self, datos):               # Publicar los datos computados en el nodo
        self.pub_control.publish(datos)

    def cmd_actuadores_callback(self, msg : cpr.CmdActuadores):
        # Extraer la información de velocidad y dirección
        self.vel_lineal_deseada = msg.vel_lineal      # Magnitud de la velocidad lineal deseada

        if self.filtrar_vel == True:
            self.vel_des_filtrada = self.filter_EMA(0.5 , self.vel_lineal_deseada, self.vel_des_filtrada)
        self.steering_deseado = msg.steering          # Magnitud ángulo de giro de las ruedas deseado
        
    def state_callback(self, state_msg : air.CarState):
        # Extraer la información del estado del coche
        self.vel_lineal_actual = state_msg.speed              # Magnitud de la velocidad actual del coche
        self.rpm_actual = state_msg.rpm 

        self.vel_act_filtrada = self.filter_EMA(0.5 , self.vel_lineal_actual, self.vel_act_filtrada)

        self.T_real = time.perf_counter() - self.now          # Periodo de tiempo real de medición
        self.now = time.perf_counter()  

        rospy.loginfo(f"Periodo real = {self.T_real}")

        self.control_callback()

    def control_callback(self):
    #def control_callback(self,timerEvent):
        if self.vel_lineal_actual is not None:
            rospy.loginfo(f"Car's current speed is: {self.vel_lineal_actual} m/s") 
            rospy.loginfo(f"Car's rpm are: {self.rpm_actual}") 

            #Prueba- medicion acceleracion real conseguida
            #current_time = rospy.Time.now().to_sec()  
            #dt = current_time - self.previous_time
            #if dt > 0:
            #    acc_real = (self.vel_lineal_actual -  self.previous_velocity)/dt
            #    rospy.loginfo(f"Car's true acceleration is: {acc_real} m/s") 
            #self.previous_velocity = self.vel_lineal_actual
            #self.previous_time = current_time
            
            # Contrucción los mensaje de control de los actuadores
            control_msg = air.CarControls() 
            now = rospy.Time.now()
            control_msg.header.stamp = now          
            control_msg.manual = False
            control_msg.manual_gear = 0
            control_msg.handbrake = False
            control_msg.gear_immediate = True

            # Cálculo de dirección. Negativo para pasar de ENU a NED. Además, se normaliza entre -1 y 1
            control_msg.steering = -self.steering_deseado / (math.pi/4) 
             

            #Calculo de señal de control
            if self.filtrar_vel == True:     
                acceleration = self.pid(self.vel_des_filtrada,self.vel_act_filtrada)
            else:
                acceleration = self.pid(self.vel_lineal_deseada,self.vel_lineal_actual)

            if self.filtrar_acc == True:
                self.acc_filtrada = self.filter_EMA(0.5,acceleration,self.acc_filtrada)
                if self.acc_filtrada >= 0:
                    control_msg.throttle = self.acc_filtrada
                    control_msg.brake = 0.0
                else:   
                    control_msg.brake = - self.acc_filtrada         #brake necesita ser positivo
                    control_msg.throttle = 0.0
            else:
                if acceleration >= 0:
                    control_msg.throttle = acceleration
                    control_msg.brake = 0.0
                else:
                    control_msg.brake = - acceleration
                    control_msg.throttle = 0.0

            

            #debuging -------------
            debug_msg=cpr.debuging()
            debug_msg.header.stamp = now
            debug_msg.original = acceleration 
            debug_msg.filtrada = self.acc_filtrada
            #debug_msg.periodo_real = self.T_real
            self.pub_debug.publish(debug_msg)
            #debuging----------

            rospy.loginfo(f"""Información enviada a los actuadores:
                            Throttle = {control_msg.throttle} 
                            Brake = {control_msg.brake} 
                            Steering = {control_msg.steering}""")
        
            # Publicar los comandos
            self.publish(control_msg)

    def pid(self,v_des:float,v_act:float)->float:               #Implementacón de un controlador PID
        #Aproximación de Euler II incremental
        q0=self.Kp*(1+(self.T_real/self.Ti)+(self.Td/self.T_real))
        q1=self.Kp*(-1-2*(self.Td/self.T_real))
        q2=self.Kp*self.Td/self.T_real

        #Calculo del error
        ek = v_des - v_act
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

    def filter_EMA(self,alpha :float, X:float, Xf_1:float) -> float:
        
        Xf = alpha*X + (1-alpha)*Xf_1

        return Xf 

if __name__ == '__main__':
    # Crear instancia del nodo
    controller = ControladorActuadores()
    # Correr el nodo
    controller.run()
    