# Paquete inicial ROS Proyecto CPR
En la siguiente explicación he sido a veces un poco repetitivo, pero es para aclarar mejor las cosas.

## Paso 1. Instalación del paquete:
Importante haber hecho previamente lo explicado en el README.md principal de este repositorio.

1. Hacer "git clone https://github.com/JoseLopez36/ProyectoCPR/tree/main" en la carpeta donde se quiera descargar el repo en local.
2. Meter "proyecto_cpr" (osea el paquete), que está dentro del repo, en el sistema de archivos de Linux en: /root/AirSim/ros/src. Aquí es donde en principio vamos a modificar el paquete añadiendo nodos y tal.
3. Abrir la ventana de comandos de Ubuntu, ir a /root/Airsim/ros y ejecutar:
```bash
sudo apt install ros-noetic-depth-image-proc
catkin_make
source devel/setup.bash 
```
El primer comando es para instalar un paquete que podría ser necesario y que probablemente no tenéis instalado.

Este es el paquete sobre el que vamos a trabajar en el proyecto. Tiene ya las siguientes cosas instaladas/creadas:
- Librerías y "sub"-paquetes básicos. Estos los podéis ver en el archivo "package.xml" y buscar información sobre ellos en la Wiki de ROS. Si fuera necesario otros, se pueden instalar fácilmente.
- Un archivo llamado "proyecto_cpr.launch" que será el .launch principal del proyecto. Desde él se llama al paquete de AirSim para que comience a leer los datos de la simulación. También inicia RVIZ para ver el mapeado y tal en tiempo real como hemos visto en las prácticas. IMPORTANTE: desde este .launch se llamará a futuros .launch desde los que se lanzarán los diferentes nodos que creemos, osea, que habrá que ir modificándolo.

## Paso 2. Forma de trabajar:
Importante tener en cuenta todo lo siguiente.

Para ir actualizando este repositorio, cuando añadais y modifiqueis cosas en el paquete, como con los comandos de Git se sube todo lo modificado en la carpeta "ProyectoCPR" (osea el repo), tendréis que copiar la carpeta "proyecto_cpr" (osea el paquete) desde los archivos de Ubuntu y pegarla en la carpeta donde clonarais el repo en el Paso1 (reemplazando la versión sin modificar).

Otra cosa muy importante es que CADA UNO TENDRÁ SU PROPIA RAMA A DONDE SUBIRÁ SUS AVANCES Y MODIFICACIONES. Por favor que nadie se equivoque y suba lo que haya cambiado a la rama principal o a la de otro, por si acaso.
