# ProyectoCPR

# Instalación de AirSim con ROS en WSL2 (Ubuntu 20.04)

## Instalación

### Paso 1. Instalar WSL con Ubuntu-20.04:
```bash
wsl --install -d Ubuntu-20.04
```

### Paso 2. Instalar ROS Noetic en Ubuntu:
- Abre Ubuntu-20.04 desde el menú de aplicaciones de Windows.
- Ejecuta los siguientes comandos para instalar ROS Noetic:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop python3-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Comprobación:
Para verificar que ROS Noetic se ha instalado correctamente, ejecuta los siguientes comandos:
```bash
cd
roscore
```
Si **roscore** se inicia correctamente, ROS Noetic está listo para usarse.

### Paso 3. Construir AirSim:
Ejecuta los siguientes comandos para instalar dependencias y construir AirSim:
```bash
sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-mavros*
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
./setup.sh
./build.sh
cd ros
catkin_make
export WSL_HOST_IP=$(ip route | grep default | awk '{print $3}')
```
### Paso 4. Crear la configuración de AirSim:
- Crea una carpeta llamada AirSim en C:\Users\username\Documents.
- Copia tu archivo settings.json en esa carpeta.
- Modifica settings.json si es necesario, teniendo cuidado con los cambios, ya que aquí puedes ajustar sensores y otras configuraciones del vehículo.

---

## Uso

### Paso 1. Lanzar el ejecutable del proyecto:
Ejecuta el .exe correspondiente del proyecto. Los ejecutables se encuentran en los Releases.

### Paso 2. Ejecutar AirSim con ROS desde WSL:
Abre la consola de WSL (buscando "Ubuntu-20.04" en el menú de Windows) y ejecuta los siguientes comandos:
```bash
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
```
### Opcional
```bash
roslaunch airsim_ros_pkgs rviz.launch
```


