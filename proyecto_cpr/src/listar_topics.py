#!/usr/bin/env python3

import rospy

# Inicializa el nodo
rospy.init_node('listar_topics_activos')

# Obtiene información de todos los topics activos
topics = rospy.get_published_topics()

# Imprime todos los topics activos
print("Topics activos:")
for topic, msg_type in topics:
    print(f"Topic: {topic} - Tipo de mensaje: {msg_type}")

