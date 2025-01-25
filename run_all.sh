#!/bin/bash

# Argumentos
LAUNCHER=${1:-"proyecto_cpr.launch"}
LAUNCH_RVIZ=${2:-"false"}      
RVIZ_CONFIG=${3:-"default"}   
WSL_IP=${4:-"host.docker.internal"}

# Configurar la variable de entorno DISPLAY
export DISPLAY="$WSL_IP:0.0"
echo "DISPLAY=$DISPLAY"

# Cambiar al directorio del proyecto ROS
cd ~/ProyectoCPR/ros || { echo "Directorio ~/ProyectoCPR/ros no encontrado"; exit 1; }

# Fuente del entorno ROS
source devel/setup.bash

# ------------------------ AirSim ------------------------

# Lanzar wrapper ROS de AirSim
echo "Lanzando AirSim ROS node..."
roslaunch airsim_ros_pkgs airsim_node.launch \
    host:=$WSL_IP \
    > /dev/null 2>&1 &

# Espera unos segundos para que AirSim arranque
sleep 5

# ------------------------- RVIZ -------------------------

# Lanzar RViz
if [ "$LAUNCH_RVIZ" = "true" ] || [ "$LAUNCH_RVIZ" = "True" ]; then
    echo "Lanzando RViz con config: $RVIZ_CONFIG"
    roslaunch proyecto_cpr_pkg rviz.launch \
        config:=$RVIZ_CONFIG \
        > /dev/null 2>&1 &

    # Espera unos segundos para que RViz arranque
    sleep 3
else
    echo "Omitiendo RViz..."
fi

# ----------------------- Proyecto -----------------------

# Lanzar Proyecto CPR (en primer plano mostrando salida)
echo "Lanzando Proyecto CPR..."
roslaunch proyecto_cpr_pkg $LAUNCHER

