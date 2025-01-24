#!/bin/bash

# Argumentos
WSL_IP=${1:-"host.docker.internal"}

# Configurar la variable de entorno DISPLAY
export DISPLAY="$WSL_IP:0.0"
echo "DISPLAY=$DISPLAY"

# Cambiar al directorio del proyecto ROS
cd ~/ProyectoCPR/ros || { echo "Directorio ~/ProyectoCPR/ros no encontrado"; exit 1; }

# Fuente del entorno ROS
source devel/setup.bash

# Lanzar PlotJuggler
rosrun plotjuggler plotjuggler