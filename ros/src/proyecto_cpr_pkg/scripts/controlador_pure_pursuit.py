#!/usr/bin/env python3

import math
from typing import NamedTuple

import rospy
import tf
import geometry_msgs.msg as geo
import std_msgs.msg as std
import nav_msgs.msg as nav
import visualization_msgs.msg as vis
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr

class VehicleParams(NamedTuple):
    wheelbase   : float                 # Longitud entre ejes [m]
    max_speed   : float                 # Velocidad máxima [m/s]
    max_acc     : float                 # Aceleración máxima [m/s^2]
    max_brake   : float                 # Capacidad de frenada máxima [m/s^2]
    max_lat_acc : float                 # Aceleración lateral máxima tolerada [m/s^2]
    max_steer   : float                 # Angulo de dirección máximo [rad]
    max_steer_change : float            # Velocidad angular del ángulo de dirección máxima [rad/s]

class AlgorithmParams(NamedTuple):
    min_lookahead_dist      : float     # Distancia de lookahead mínima [m]
    max_lookahead_dist      : float     # Distancia de lookahead máxima [m]
    N                       : int       # Número de puntos tomados del camino hacia delante y detrás del punto más cercano al vehículo

class ControladorPurePursuit:
    def __init__(self):
        # Variables miembro
        # Entrada
        self.current_pose = None    # (geometry_msgs/PoseStamped)
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.path = None            # (nav_msgs/Path)
        # Salida
        self.target_speed = 0.0
        self.target_steering = 0.0
        self.lookahead_point = None

        # Parámetros
        self.vehicle_params = VehicleParams(
            wheelbase = 2.7,        # Distancia entre ejes [m], típico de un turismo compacto/medio
            max_speed = 15.0,       # ~54 km/h, velocidad máxima aproximada en ciudad
            max_acc = 2.0,          # Aceleración máxima [m/s^2] (moderada para ciudad)
            max_brake = 4.0,        # Frenada máxima [m/s^2] (más alta para seguridad)
            max_lat_acc = 3.0,      # Aceleración lateral máxima [m/s^2]
            max_steer = 0.61,       # ~35° en rad (recomendado: 30° a 40°)
            max_steer_change = 0.52 # ~30°/s en rad/s (recomendado: 20°/s a 120°/s)
        )
        self.algo_params = AlgorithmParams(
            min_lookahead_dist = 3.0,  # [m] distancia mínima de lookahead
            max_lookahead_dist = 15.0, # [m] distancia máxima de lookahead
            N = 40
        )

        # Transform Listener
        self.tf_listener = tf.TransformListener()

        # Suscriptores
        rospy.Subscriber('/airsim_node/car_1/car_state', air.CarState, self.state_callback)
        rospy.Subscriber('/airsim_node/car_1/car_cmd', air.CarControls, self.controls_callback)
        rospy.Subscriber('/trayectoria_local', nav.Path, self.path_callback)

        # Publicadores
        self.pub_actuadores = rospy.Publisher(
            '/cmd_actuadores', 
            cpr.CmdActuadores,
            queue_size=10
        )
        self.pub_visual = rospy.Publisher(
            '/pure_pursuit_vis', 
            vis.Marker,
            queue_size=10
        )

        rospy.loginfo("Controlador Pure Pursuit inicializado.")

    def state_callback(self, msg):
        # Extraer la información del estado del coche
        self.current_pose = geo.PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose  # Pose actual del coche
        self.current_speed = msg.speed          # Velocidad actual del coche

    def controls_callback(self, msg):
        # Extraer la información de los controles del coche
        self.current_steering = -msg.steering * (math.pi/4)  # Negativo para pasar de NED a ENU. Se desnormaliza

    def path_callback(self, msg):
        # Extraer la información de la trayectoria recibida
        self.path = msg                         # Trayectoria local a seguir

    def compute_curvature(self, path, mode=1):
        """
        Calcula la curvatura de un camino usando N puntos hacia delante y hacia atrás del vehículo
        
        Args:
            path (nav_msgs/Path): Camino evaluado.
            mode (int): 1 para curvatura máxima (seguridad), 2 para curvatura media (eficiencia).
        
        Returns:
            kappa_out (float): Valor de curvatura calculado (máxima o media según el modo).
        """
        # Verificar que el número de puntos sea válido
        n_path_points = len(path.poses)
        if n_path_points < 3:
            return 0.0

        # Calcular curvatura a partir de 3 puntos consecutivos
        kappas = []  # Lista para almacenar las curvaturas
        for i in range(0, n_path_points - 2):
            # Obtener los puntos consecutivos
            p1 = path.poses[i].pose
            p2 = path.poses[i + 1].pose
            p3 = path.poses[i + 2].pose

            # Extraer coordenadas x, y
            x1, y1 = p1.position.x, p1.position.y
            x2, y2 = p2.position.x, p2.position.y
            x3, y3 = p3.position.x, p3.position.y

            # Calcular la curvatura
            # Distancias entre pares de puntos
            d12 = math.hypot(x2 - x1, y2 - y1)
            d23 = math.hypot(x3 - x2, y3 - y2)
            d13 = math.hypot(x3 - x1, y3 - y1)
            # Evitar divisiones por cero
            if d12 < 1e-9 or d23 < 1e-9 or d13 < 1e-9:
                curv = 0.0
            else:
                # Área del triángulo
                area = 0.5 * abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))

                # Calcular curvatura
                if area < 1e-9:
                    curv = 0.0
                else:
                    curv = (4.0 * area) / (d12 * d23 * d13)

            # Almacenar el valor absoluto de la curvatura
            curv = min(curv, 1.0) # Se satura
            kappas.append(abs(curv))  

        if len(kappas) == 0:
            return 0.0
        
        if mode == 1:
            # Modo de curvatura máxima
            kappa_out = max(kappas)
        elif mode == 2:
            # Modo de curvatura media
            kappa_out = sum(kappas) /len(kappas)
        else:
            # Modo por defecto (seguridad)
            kappa_out = max(kappas)

        return kappa_out

    def trapezoidal_smoothing(self, curr_val, target_val, max_incr, max_drec):
        """
        Aplica suavizado trapezoidal para acercar el valor actual al valor objetivo 
        con aceleración y deceleración máximas.

        Args:
            curr_val (float): El valor actual del comando (e.g., velocidad actual).
            target_val (float): El valor objetivo hacia el que se quiere acercar.
            max_incr (float): Aceleración máxima por segundo (en valor absoluto).
            max_drec (float): Deceleración máxima por segundo (en valor absoluto).

        Returns:
            out(float): El nuevo valor suavizado.
        """
        diff = target_val - curr_val

        # Calcular aceleración/deceleración necesaria para alcanzar el objetivo
        if diff > 0:
            accel = min(diff / self.T, max_incr)
            out = curr_val + accel * self.T
        else:
            decel = min(-diff / self.T, max_drec)
            out = curr_val - decel * self.T

        # Ajustar para asegurarse de no pasar el objetivo
        if diff > 0:
            return min(out, target_val)
        else:
            return max(out, target_val)
    
    def speed_planning(self, kappa, curr_speed, max_speed=10.0, max_acceleration=3.0, max_deceleration=3.0, max_lat_acc=3.0):
        """
        Calcula la velocidad objetivo del vehículo en función de la curvatura del camino.

        Args:
            kappa (float): Curvatura del camino.
            curr_speed (float): Velocidad actual del vehículo (m/s).
            max_speed (float): Velocidad máxima permitida (m/s).
            max_acceleration (float): Aceleración máxima permitida (m/s²).
            max_deceleration (float): Frenada máxima permitida (m/s²).
            max_lat_acc (float): Máxima aceleración lateral segura (m/s²).

        Returns:
            smooth_target_speed (float): Velocidad objetivo suavizada del vehículo (m/s).
        """
        # Calcular el radio del círculo en función de la curvatura
        if kappa < 1e-6:  
            # Si la curvatura es muy pequeña, se asume que es un camino recto
            target_speed = max_speed
        else:
            # Si la curvatura NO es muy pequeña, se calcula la velocidad segura
            target_speed = math.sqrt((1.0 / kappa) * max_lat_acc)

        # Limitar la velocidad al valor máximo permitido
        target_speed = min(target_speed, max_speed)

        # Aplicar suavizado para mejorar la respuesta del vehículo ante cambios bruscos
        smooth_target_speed = self.trapezoidal_smoothing(curr_speed, target_speed, max_acceleration, max_deceleration)

        return smooth_target_speed
    
    def compute_adaptive_lookahead_dist(self, kappa, current_speed, max_speed, min_lookahead_dist, max_lookahead_dist, alpha=0.2, beta=0.25):
        """
        Calcula la distancia de lookahead adaptativa teniendo en cuenta tanto la
        curvatura de la trayectoria como la velocidad actual del vehículo.

        Args:
            kappa (float): Curvatura del camino.
            current_speed (float): Velocidad actual del vehículo (m/s).
            max_speed (float): Velocidad máxima permitida (m/s).
            min_lookahead_dist (float): Distancia mínima de lookahead (m).
            max_lookahead_dist (float): Distancia máxima de lookahead (m).
            alpha (float): Factor de decaimiento exponencial para kappa * speed.
            beta (float): Ponderación de la parte de velocidad. El resto (1-beta) se aplica a la parte de curvatura.
            
        Returns:
            dist (float): Distancia de lookahead computada.
        """
        # Factor de curvatura
        # Cuanto mayor sea kappa*speed, más pequeño será curv_factor.
        # alpha controla la sensibilidad al producto kappa*speed.
        #   - Si kappa*speed = 0  --> curv_factor = 1
        #   - Si kappa*speed = 5  --> curv_factor = e^(-alpha*5). 
        #   Si alpha=0.2, curv_factor ≈ e^(-1.0) = 0.3679
        K_curv = 10
        curv_factor = math.exp(-alpha * (kappa * K_curv) * current_speed)

        # Factor de velocidad
        #    current_speed = 0   --> factor_speed = 0
        #    current_speed = max --> factor_speed = 1
        speed_factor = max(0.0, min(current_speed / max_speed, 1.0))
        
        # Combinar factores y escalar al rango [min_lookahead_dist, max_lookahead_dist]
        factor = beta * speed_factor + (1.0 - beta) * curv_factor
        dist = min_lookahead_dist + factor * (max_lookahead_dist - min_lookahead_dist)

        # Restringir la distancia dentro de los límites
        dist = max(min_lookahead_dist, min(dist, max_lookahead_dist))

        return dist
    
    def find_lookahead_point(self, path, closest_idx, lookahead_dist):
        """
        Busca un punto en la trayectoria a una distancia aproximada lookahead_dist
        desde la posición actual del vehículo. Para ello:
        
        1. Encuentra el índice del path más cercano al vehículo (closest_idx).
        2. A partir de ese índice, acumula distancias entre cada par de puntos sucesivos
        hasta superar lookahead_dist.
        3. Interpola (si es necesario) para obtener la posición final exacta.

        Args:
            path (nav_msgs/Path): Trayectoria local a seguir.
            closest_idx (int): Índice del punto del camino más cercano al vehículo.
            lookahead_dist (float): Distancia de lookahead deseada [m].

        Returns:
            lookahead_point (geometry_msgs/Point): Punto en la trayectoria a ~ lookahead_dist del vehículo.
        """
        # 1. Si la trayectoria está vacía o tiene un solo punto, devolvemos el único que existe:
        if not path or len(path.poses) == 0:
            return geo.Point(0.0, 0.0, 0.0)
        if len(path.poses) == 1:
            return path.poses[0].pose.position

        # 2. Recorremos el path desde el punto más cercano hacia adelante, acumulando distancias
        dist_acumulada = 0.0
        prev_x = path.poses[closest_idx].pose.position.x
        prev_y = path.poses[closest_idx].pose.position.y

        # Si el vehículo ya está más cerca del final, ajustar si es necesario
        if closest_idx == len(path.poses) - 1:
            # Estamos en el último punto de la trayectoria
            return geo.Point(prev_x, prev_y, 0.0)

        # 3. Avanzamos sobre la trayectoria para encontrar dónde se supera lookahead_dist
        for i in range(closest_idx + 1, len(path.poses)):
            curr_x = path.poses[i].pose.position.x
            curr_y = path.poses[i].pose.position.y

            # Distancia entre el punto anterior y el actual
            segment_length = math.hypot(curr_x - prev_x, curr_y - prev_y)

            # Si al sumar segment_length superamos lookahead_dist, hay que interpolar
            if dist_acumulada + segment_length >= lookahead_dist:
                # cuánto nos falta para llegar justo a lookahead_dist
                faltante = lookahead_dist - dist_acumulada  
                # factor de interpolación en [0,1]
                t = faltante / segment_length if segment_length > 1e-9 else 0.0

                # Interpolamos
                lookahead_x = prev_x + t * (curr_x - prev_x)
                lookahead_y = prev_y + t * (curr_y - prev_y)
                return geo.Point(lookahead_x, lookahead_y, 0.0)

            # De lo contrario, seguimos avanzando
            dist_acumulada += segment_length
            prev_x, prev_y = curr_x, curr_y

        # 4. Si se recorrió todo el path y no se superó lookahead_dist, devolvemos el último punto para evitar índices fuera de rango.
        return geo.Point(prev_x, prev_y, 0.0)

    def compute_steering(self, curr_position, target_position, curr_vehicle_yaw, curr_steering, lookahead_dist):
        """
        Calcula el ángulo de giro del vehículo y suaviza la salida aplicando un suavizado trapezoidal.

        Args:
            curr_position (geometry_msgs/Point): Posición actual del vehículo en el marco del path [m].
            target_position (geometry_msgs/Point): Posición actual del punto objetivo en el marco del path [m].
            curr_vehicle_yaw (float): Orientación actual del vehículo [rad].
            curr_steering (float): Ángulo de dirección actual del vehículo [rad].
            lookahead_dist(float): Distancia de lookahead deseada [m].

        Returns:
            smooth_target_steering (float): Ángulo de dirección objetivo suavizado [rad].
        """                                                                                                         
        # Calcular el ángulo entre la dirección actual y la dirección hacia el punto
        alpha = math.atan2(
            target_position.y - curr_position.y, 
            target_position.x - curr_position.x
            ) - curr_vehicle_yaw

        # Calcular el ángulo de dirección (steering) mediante la fórmula de Pure Pursuit
        target_steering = math.atan2(2.0 * self.vehicle_params.wheelbase * math.sin(alpha), lookahead_dist)

        # Ajustar al rango permitido
        target_steering = min(max(target_steering, -self.vehicle_params.max_steer), self.vehicle_params.max_steer)

        # Aplicar suavizado para mejorar la respuesta del vehículo ante cambios bruscos
        smooth_target_steering = self.trapezoidal_smoothing(
            curr_steering,                         # Ángulo actual de dirección
            target_steering,                       # Ángulo objetivo
            self.vehicle_params.max_steer_change,  # Tasa máxima de cambio (subida)
            self.vehicle_params.max_steer_change   # Tasa máxima de cambio (bajada)
        )

        return smooth_target_steering

    def run_pure_pursuit(self):
        """
        Planifica la velocidad del vehículo para una conducción segura y eficiente.
        Ejecuta un algoritmo de Pure Pursuit modificado con adaptabilidad a la curvatura del camino.
        """        
        # Verificar si el camino está disponible y es válido
        error = False
        if not self.path or len(self.path.poses) == 0:
            rospy.logwarn("ControladorPurePursuit: No hay path disponible. Esperando a que se reciba un camino...")
            error = True
        elif len(self.path.poses) < 5:
            rospy.logwarn("ControladorPurePursuit: El path recibido no posee suficientes puntos.")
            error = True
        
        # Verificar si el estado del vehículo es válido
        if not self.current_pose and not error:
            rospy.logwarn("ControladorPurePursuit: Estado del vehiculo no valido.")
            error = True
        elif self.current_pose and not error:
            # Extraer la pose del vehículo
            try:
                # Transformar al marco de la trayectoria
                self.current_pose.header.stamp = rospy.Time(0) # Se fuerza a obtener la última transformación disponible
                transformed_pose = self.tf_listener.transformPose(self.path.header.frame_id, self.current_pose)
                # Extraer posición del vehículo en el marco transformado
                vehicle_pos = transformed_pose.pose.position
                # Extraer yaw del cuaternión en el marco transformado
                quaternion = [
                    transformed_pose.pose.orientation.x,
                    transformed_pose.pose.orientation.y,
                    transformed_pose.pose.orientation.z,
                    transformed_pose.pose.orientation.w,
                ]
                _, _, vehicle_yaw = tf.transformations.euler_from_quaternion(quaternion)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("ControladorPurePursuit: Error al transformar la pose del vehículo: %s", e)
                error = True
        
        # Si ha habido algún error, frenar inmediatamente
        if error:
            cmd = cpr.CmdActuadores()
            cmd.vel_lineal = 0.0
            cmd.steering = 0.0
            self.publish(cmd, None)
            return
        
        # PASO 1: Reducir la trayectoria a N puntos hacia delante y hacia atrás del punto más cercano al vehículo
        # Hallar el índice del punto más cercano en la trayectoria
        min_dist_sq = float('inf')
        closest_idx = 0
        for i, pose_stamped in enumerate(self.path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist_sq = (px - vehicle_pos.x) ** 2 + (py - vehicle_pos.y) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_idx = i
        # Crear un nuevo Path con los puntos recortados
        reduced_path = nav.Path()
        start_idx = max(closest_idx - self.algo_params.N, 0)
        end_idx   = min(closest_idx + self.algo_params.N, len(self.path.poses))
        reduced_path.header = self.path.header
        reduced_path.poses  = self.path.poses[start_idx:end_idx]

        # PASO 2: Calcular la curvatura del camino. Para ello se proponen 2 estrategias:
        # - Modo 1: Curvatura máxima de N puntos del camino (más segura)
        # - Modo 2: Curvatura media de N puntos del camino (más eficiente)
        curvature_mode = 1
        kappa = self.compute_curvature(
            reduced_path, 
            curvature_mode
        )

        # Planificar la velocidad para asegurar una conducción segura y eficiente. Para ello se utiliza:
        # - Curvatura del camino (kappa)
        # - Máxima capacidad de frenada del vehículo
        self.target_speed = self.speed_planning(
            kappa,
            self.current_speed, 
            self.vehicle_params.max_speed,
            self.vehicle_params.max_acc, 
            self.vehicle_params.max_brake,
            self.vehicle_params.max_lat_acc
        )

        # Aplicar controlador Pure Pursuit adaptativo
        # Distancia de lookahead adaptativa
        lookahead_dist = self.compute_adaptive_lookahead_dist(
            kappa, 
            self.current_speed,
            self.vehicle_params.max_speed,
            self.algo_params.min_lookahead_dist,
            self.algo_params.max_lookahead_dist
        )
        # Pure Pursuit
        self.lookahead_point = self.find_lookahead_point(
            self.path, 
            closest_idx,
            lookahead_dist
        )
        # Cálculo del ángulo de dirección
        self.target_steering = self.compute_steering(
            vehicle_pos, 
            self.lookahead_point, 
            vehicle_yaw, 
            self.current_steering, 
            lookahead_dist
        )

        # Construir los comandos para el controlador de bajo nivel
        cmd = cpr.CmdActuadores()
        # Datos
        cmd.vel_lineal = self.target_speed
        cmd.steering = self.target_steering
        # Header
        now = rospy.Time.now()
        cmd.header.stamp = now

        # Construir el punto de lookahead para visualización en rviz
        lookahead_point_marker = vis.Marker()
        # Datos
        lookahead_point_marker.pose.position = self.lookahead_point
        lookahead_point_marker.pose.orientation.w = 1.0
        # Visualización
        lookahead_point_marker.type = vis.Marker.SPHERE
        lookahead_point_marker.action = vis.Marker.ADD
        lookahead_point_marker.scale.x = 5  
        lookahead_point_marker.scale.y = 5
        lookahead_point_marker.scale.z = 5
        lookahead_point_marker.color.a = 1.0  
        lookahead_point_marker.color.r = 1.0 
        lookahead_point_marker.color.g = 0.0
        lookahead_point_marker.color.b = 0.0
        # Header
        lookahead_point_marker.header = self.path.header
        lookahead_point_marker.header.stamp = now
        # Otros
        lookahead_point_marker.ns = "lookahead_point"
        lookahead_point_marker.id = 0

        # Publicar los comandos 
        self.publish(cmd, lookahead_point_marker)

        # Notificar
        rospy.loginfo(
            f"Comandos publicados: Velocidad lineal = {cmd.vel_lineal:.2f} m/s, Direccion = {cmd.steering:.2f} rad, Timestamp = {cmd.header.stamp.to_sec():.2f}s"
        )

    def publish(self, datos, marker):
        # Publicar los datos computados en el nodo
        if datos:
            self.pub_actuadores.publish(datos)
        if marker:
            self.pub_visual.publish(marker)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(10)  # 10 Hz
        self.T = 1/10          # Tiempo entre iteraciones
        while not rospy.is_shutdown():
            self.run_pure_pursuit()
            rate.sleep()

if __name__ == '__main__':
    # Inicializar el nodo
    rospy.init_node('controlador_pure_pursuit')

    # Crear objeto
    nodo = ControladorPurePursuit()

    # Correr el nodo
    nodo.run()
