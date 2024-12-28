#!/usr/bin/env python3

import rospy

# Especifica el tipo de mensaje que deseas buscar (por ejemplo, "airsim_ros_pkgs/VelCmd")
tipo_mensaje_objetivo = "airsim_ros_pkgs/VelCmd"

# Inicializa el nodo
rospy.init_node('buscar_topics')

# Obtiene información de todos los topics activos
topics = rospy.get_published_topics()

# Filtra y muestra los topics que coincidan con el tipo de mensaje especificado
for topic, msg_type in topics:
    if msg_type == tipo_mensaje_objetivo:
        print(f"Topic: {topic} - Tipo de mensaje: {msg_type}")