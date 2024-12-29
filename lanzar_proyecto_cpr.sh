#!/bin/bash

# Asignar manualmente la IP
wsl_host_ip="172.28.192.1"

# Verificar si se obtuvo una IP válida
if [[ -z "$wsl_host_ip" ]]; then
    echo "Error: No se pudo obtener la IP del adaptador WSL 'vEthernet'."
    exit 1
fi

echo "IP de WSL utilizada: $wsl_host_ip"

# Cambiar al directorio del proyecto ROS
cd ~/ProyectoCPR/ros || { echo "Directorio ~/ProyectoCPR/ros no encontrado"; exit 1; }

# Compilar el proyecto ROS
catkin_make || { echo "Error al compilar el proyecto con catkin_make."; exit 1; }

# Fuente del entorno ROS
source devel/setup.bash

# Ejecutar AirSim con ROS
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$wsl_host_ip
