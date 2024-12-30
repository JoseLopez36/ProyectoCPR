#!/bin/bash

# Verificar si se proporcionó un argumento para la configuración de RViz
if [[ -z "$1" ]]; then
    rviz_config="default"
    echo "No se proporcionó un archivo de configuración para RViz. Usando configuración predeterminada: $rviz_config"
else
    rviz_config="$1"
    echo "Usando archivo de configuración de RViz: $rviz_config"
fi

# Cambiar al directorio del proyecto ROS
cd ~/ProyectoCPR/ros || { echo "Directorio ~/ProyectoCPR/ros no encontrado"; exit 1; }

# Fuente del entorno ROS
source devel/setup.bash

# Lanzar RViz
source /tmp/shared_env_vars # Recargar la variable DISPLAY
roslaunch proyecto_cpr_pkg rviz.launch config:=$rviz_config