FROM ubuntu:20.04

# Instalar dependencias iniciales
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    apt-utils \
    apt-transport-https \
    software-properties-common \
    sudo \
    tzdata \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* && \
    dpkg-reconfigure -f noninteractive tzdata

# Crear un usuario para trabajar
ARG UNAME=testuser
RUN adduser --disabled-password --gecos '' $UNAME
RUN adduser $UNAME sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Copiar y ejecutar el script de instalación de ROS Noetic
ADD install_ros_deps.sh /
RUN chmod +x /install_ros_deps.sh
RUN DEBIAN_FRONTEND=noninteractive /install_ros_deps.sh

# Instalar dependencias adicionales de ROS
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    ros-noetic-pcl-ros \
    # *************** Anadir aqui mas paquetes *************** \   
    # ******************************************************** \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Configuración del entorno para ROS
USER $UNAME
WORKDIR /home/${UNAME}
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc