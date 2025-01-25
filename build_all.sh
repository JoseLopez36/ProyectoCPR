#!/bin/bash

# Cambiar al directorio del proyecto ROS
cd ~/ProyectoCPR/ros || { echo "Directorio ~/ProyectoCPR/ros no encontrado"; exit 1; }

# Fuente del entorno ROS
source devel/setup.bash

# Compilar el proyecto ROS
echo "Compilando el workspace con catkin_make..."
catkin_make || { 
    echo "Error al compilar el proyecto con catkin_make."; 
    exit 1; 
}

# Fuente del entorno ROS
source devel/setup.bash
