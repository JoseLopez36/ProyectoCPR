#!/bin/bash

# Cambiar al directorio del proyecto ROS
cd ~/ProyectoCPR/ros || { echo "Directorio ~/ProyectoCPR/ros no encontrado"; exit 1; }

# Fuente del entorno ROS
source devel/setup.bash

# Ejecutar proyecto CPR
roslaunch proyecto_cpr_pkg proyecto_cpr.launch