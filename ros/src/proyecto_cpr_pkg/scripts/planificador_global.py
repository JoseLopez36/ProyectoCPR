#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
import std_msgs.msg as std
import nav_msgs.msg as nav
import airsim_ros_pkgs.msg as air
import proyecto_cpr_pkg.msg as cpr
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
import numpy as np
import cv2

# Variables globales
win=(30,160) # 140 190 #20 98

image=r"/home/testuser/ProyectoCPR/ros/src/proyecto_cpr_pkg/config/mapa_bin.png"
imagen = cv2.imread(image)

gray = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

_,ibin= cv2.threshold(gray, 70, 1, cv2.THRESH_BINARY)

laberinto2=1-ibin

ANCHO_VENTANA=985
ALTO_VENTANA=745

tam=5

negro=(0,0,0)
blanco = (255, 255, 255)
azul = (0, 0, 255)
rojo= (255,0,0)
verde_cesped = (34, 139, 34)
gris_carretera = (50, 50, 50)
amarillo=(255, 255, 0)

def smooth_traj(trayectoria, factor_suavizado=2.0, densidad_puntos=10):
    # Extraer solo coordenadas x e y (z es fijo según tu implementación original)
    x = []
    y = []
    for pose in trayectoria.poses:
        x.append(pose.pose.position.x)
        y.append(pose.pose.position.y)
    
    x = np.array(x)
    y = np.array(y)
    
    # Calcular distancia acumulada como parámetro
    dx = np.diff(x)
    dy = np.diff(y)
    distancias = np.sqrt(dx**2 + dy**2)
    distancias_acumuladas = np.cumsum(distancias)
    distancias_acumuladas = np.insert(distancias_acumuladas, 0, 0)
    
    # Splines de suavizado
    spline_x = UnivariateSpline(
        distancias_acumuladas, 
        x, 
        s=factor_suavizado * len(x)
    )
    
    spline_y = UnivariateSpline(
        distancias_acumuladas,
        y,
        s=factor_suavizado * len(y)
    )
    
    # Generar nueva distribución de puntos
    nuevos_puntos = np.linspace(
        0, 
        distancias_acumuladas[-1], 
        num=densidad_puntos * len(distancias_acumuladas)
    )
    
    nuevos_x = spline_x(nuevos_puntos)
    nuevos_y = spline_y(nuevos_puntos)
    
    # Construir trayectoria suavizada (versión corregida)
    trayectoria_suavizada = nav.Path()
    trayectoria_suavizada.header = trayectoria.header
    trayectoria_suavizada.poses = []
    
    for xi, yi in zip(nuevos_x, nuevos_y):
        pose = geo.PoseStamped()
        pose.header.frame_id = trayectoria.header.frame_id
        pose.pose.position.x = xi
        pose.pose.position.y = yi
        pose.pose.position.z = 0.0  # Z fijo como en tu código original
        pose.pose.orientation.w = 1.0  # Orientación neutra
        trayectoria_suavizada.poses.append(pose)
    
    return trayectoria_suavizada

def expand_path(x_coords, y_coords, num_points=1000):
    # Crear un rango de índices para los puntos originales
    original_indices = np.linspace(0, 1, len(x_coords))
    
    # Crear un rango de índices para los puntos interpolados
    new_indices = np.linspace(0, 1, num_points)
    
    # Interpolar los valores de x y y
    x_interpolated = interp1d(original_indices, x_coords, kind='linear')(new_indices)
    y_interpolated = interp1d(original_indices, y_coords, kind='linear')(new_indices)
    
    return x_interpolated, y_interpolated

def distancia_manhattan(pos_init,pos_final):
    px1,py1=pos_init
    px2,py2=pos_final
    d=abs((px2-px1))+abs((py2-py1))

    return d

def dibuja_grid(Ancho, Alto, tam):
    nodos = []
    for i in range(Alto//tam):
        fila = []
        for j in range(Ancho//tam):
            nodo = Nodo(i, j,laberinto2[i][j],None,0,0)
            fila.append(nodo)

        nodos.append(fila)
    return nodos

class Nodo():
    def __init__(self,posx,posy,costo,parent,star,andar):
        self.posx=posx
        self.posy=posy
        self.costo=costo
        self.parent=parent
        self.coste_a_star=star
        self.coste_andar=andar

    def action(self,accion):
        i=self.posx
        j=self.posy
        costo_extra = 1  # Costo base para movimientos ortogonales
        if accion in ["d1", "d2", "d3", "d4"]:
            costo_extra = 1.414  # sqrt(2) para movimientos diagonales
        if accion == "arriba":
            i+=1
        elif accion == "abajo":
            i-=1
        elif accion == "derecha":
            j+=1
        elif accion == "izquierda":
            j-=1

        elif accion == "d1":
            j+=1
            i+=1
        elif accion == "d2":
            j-=1
            i+=1
        elif accion == "d3":
            j-=1
            i-=1
        elif accion == "d4":
            j+=1
            i-=1

        if i < 0 or j < 0 or i >= ALTO_VENTANA // tam or j >= ANCHO_VENTANA // tam:
            return None  # Si alguna coordenada es negativa, retorna None
        return i, j, costo_extra
    def __repr__(self):
        return f"({self.posx}, {self.posy}, {self.coste_a_star})"
    
class Fronter():
    def __init__(self):
        self.frontera=[]

    def añadir(self,Nodo):
        self.frontera.append(Nodo)


    def vacia(self):
        if len(self.frontera)==0:
            return True
        return False
    
    def quitar(self,goal):
        if self.vacia():
            raise Exception("Frontera vacia")
        else:
            nodo=self.frontera[-1]
            self.frontera=self.frontera[:-1]
            return nodo

class cola_A_star(Fronter):
    def quitar(self,goal):
        if self.vacia():
            raise Exception("Frontera vacia")
        else:
            for nodos in self.frontera:
                nodos.coste_a_star=nodos.coste_andar+distancia_manhattan((nodos.posx,nodos.posy),goal)
            nodos_ordenados = sorted(self.frontera, key=lambda nodo: nodo.coste_a_star)
            self.frontera =nodos_ordenados
            nodo=self.frontera[0]
            self.frontera=self.frontera[1:]
            return nodo

actions=["d1","d2","d3","d4","arriba","abajo","izquierda","derecha"]

class PlanificadorGlobal:
    def __init__(self):
        # Variables internas
        self.simulando=True
        self.Win=False
        self.fronteras=cola_A_star()
        self.nodos=dibuja_grid(ANCHO_VENTANA,ALTO_VENTANA,tam)
        self.nodos[win[0]][win[1]].costo=0
        self.inicio=self.nodos[136][98] #1,1 #80 75
        self.inicio.coste_andar=0
        self.fronteras.añadir(self.inicio)
        self.visitados = set()
        self.visitados.add((self.inicio.posx,self.inicio.posy))
        self.x_cord=[]
        self.y_cord=[]

        # Publicadores
        self.pub_trayectoria_global = rospy.Publisher(
            '/trayectoria_global', 
            nav.Path,
            queue_size=1,
            latch=True
        )

        rospy.loginfo("Nodo de Planificador Global inicializado.")

    def ejecutar_planificador_global(self):
        if not self.simulando:
            return
        
        # Implementación del planificador global
        while not self.fronteras.vacia() and not self.Win:
            nodo=self.fronteras.quitar(win) 
            if (nodo.posx,nodo.posy)==win:
                self.Win=True
                solucion_nodo = nodo
            for a in actions:
                result=nodo.action(a)
                if result is not None:
                    i,j,costo_extra=result
                    vecino=self.nodos[i][j]
                    
                    if (i,j) not in self.visitados and vecino.costo==0:
                        self.visitados.add((i,j))
                        self.fronteras.añadir(vecino)
                        vecino.parent=nodo
                        vecino.coste_andar=nodo.coste_andar+costo_extra
        
        if self.Win:
            cont=0
            while (solucion_nodo.parent is not None):
                self.x_cord.append(solucion_nodo.posx)
                self.y_cord.append(solucion_nodo.posy)
                solucion_nodo=solucion_nodo.parent                

                cont+=1
            
            self.x_cord=self.x_cord[::-1]
            self.y_cord=self.y_cord[::-1]
            
            # Notificar
            rospy.loginfo(
                f"La solucion son {cont} pasos"
            )

            # Construir la trayectoria global
            trayectoria = nav.Path()
            trayectoria.header.frame_id = "world_enu"
            now = rospy.Time.now()
            trayectoria.header.stamp = now

            # Expandir el path
            #x_expanded, y_expanded = expand_path(self.x_cord, self.y_cord, num_points=1000)

            # Crear los puntos del camino a partir de los vectores
            for x, y in zip(self.x_cord, self.y_cord):
                y=97-y
                x=x-136

                pose = geo.PoseStamped()
                pose.header.frame_id = "world_enu"
                pose.pose.position.x = -y  
                pose.pose.position.y = -x  
                pose.pose.position.z = 0.0  # Coordenada Z fija
                pose.pose.orientation.w = 1.0  # Sin rotación
                trayectoria.poses.append(pose)

            # Suavizar esquinas
            trayectoria_suavizada = smooth_traj(
                trayectoria,
                factor_suavizado=0.1,  # Más suave
                densidad_puntos=10     # Más puntos para curvas detalladas
            )

            # Publicar la trayectoria
            self.publish(trayectoria_suavizada)

            # Parar nodo
            self.simulando=False

    def publish(self, datos):
        # Publicar los datos computados en el nodo
        self.pub_trayectoria_global.publish(datos)

    def run(self):
        # Mantener el nodo en ejecución
        rate = rospy.Rate(1.0)  # 1 Hz
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