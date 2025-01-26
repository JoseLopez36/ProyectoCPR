# ProyectoCPR: Implementación y Simulación de un Sistema de Navegación Autónoma para Vehículos Terrestres

## Instalación de AirSim junto a ROS en Docker (Windows 10 y 11)

Este repositorio contiene los archivos necesarios para configurar un contenedor Docker que ejecute `AirSim` y `ROS` en `Windows`. A continuación, se describen los pasos para su instalación y puesta en marcha. Para más información: [How to use AirSim with Robot Operating System (ROS)](https://cosys-lab.github.io/ros_python/)

### Prerequisitos

1. **Docker Desktop**  
   - [Descargar e instalar Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/)

2. **Servidor gráfico para Windows (requerido para RViz)**  
   - [Xming](http://www.straightrunning.com/XmingNotes/) (se recomiendan también las fuentes Xming-fonts)
   - Asegúrate de tenerlo instalado y en ejecución antes de utilizar RViz.

---

### Paso 1. Clonar el repositorio

Clona el repositorio y accede al directorio del proyecto.
```bash
git clone https://github.com/JoseLopez36/ProyectoCPR.git
cd ProyectoCPR
```

### Paso 2. Construir la imagen Docker

Dentro del repositorio clonado, entra a la carpeta `docker` y construye la imagen Docker:
```bash
cd docker
docker build -t airsim-ros -f Dockerfile .
```
Nota: Este paso puede tomar unos minutos.

### Paso 3. Configurar la variable de entorno
En Windows, crea la variable de entorno `CPR_PATH` con la ruta donde se haya clonado este repositorio.
Por ejemplo:
- Si clonaste el repositorio en `C:\Users\<tu-usuario>\Documents\ProyectoCPR`, entonces tu CPR_PATH deberá ser `C:\Users\<tu-usuario>\Documents\ProyectoCPR`.
Para crear la variable de entorno en Windows:
1. Presiona `Windows + R` y escribe `Sysdm.cpl`.
2. Ve a `Avanzado > Variables de entorno`
3. En Variables de sistema, crea una nueva variable con el nombre CPR_PATH y el valor con la ruta del repositorio.

### Paso 4. Iniciar el contenedor

Para iniciar el contenedor, arrancar Docker Desktop y desde el directorio raíz del proyecto:
1. Ve a la carpeta `tools`.
2. Haz doble click en el archivo `start_container.bat`.

Esto correrá el contenedor con la imagen construida anteriormente y montará el volumen correspondiente. Una vez esté corriendo, tendrás acceso a un shell dentro del contenedor. Este paso debe repetirse cada vez que se quiera iniciar el contenedor.

### Paso 5. Construir y configurar AirSim

Una vez dentro del contenedor, construye AirSim:
```bash
cd ~/ProyectoCPR/external/AirSim
./setup.sh
./build.sh
```

Para configurar AirSim es necesario realizar los siguientes pasos:
1. Crear la carpeta `AirSim ` en `Documentos`.
2. Pegar en esa carpeta el archivo `settings.json` presente en la raíz de este repositorio.
3. (opcional) Modificar archivo `settings.json` si se desea para cambiar sensores, vehículos...

### Paso 6. Construir el workspace de ROS

A continuación, procede a construir el workspace de ROS:
```bash
cd ~/ProyectoCPR/ros
catkin_make
source devel/setup.bash
```
Nota: Asegúrate de ejecutar source devel/setup.bash cada vez que abras un nuevo terminal dentro del contenedor para que las configuraciones de tu workspace estén disponibles.

### Paso 7. Configuración para usar RViz

Para usar RViz en Windows, necesitas un servidor X. Se recomienda Xming:
1. Descargar Xming y Xming-fonts de Public Domain Releases
2. Instalar ambos paquetes.
3. Ejecutar Xming antes de abrir RViz en el contenedor.

## Uso

Para lanzar el proyecto se deben seguir los siguientes pasos:

### Paso 1. Lanzar el ejecutable del proyecto:

Ejecuta el .exe correspondiente del proyecto. Los distintos ejecutables se encuentran en los Releases del repositorio.

### Paso 2. Abrir contenedor Docker:

Abre el contenedor haciendo doble-click en `tools/start_container.bat`. Si deseas abrir otro terminal para el mismo contenedor, ejecuta `tools/open_container_terminal.bat`. Si deseas cerrar el contenedor, ejecuta `tools/docker_kill.bat`.

### Paso 3. Ejecutar el programa desde el contenedor Docker:

Ejecuta los siguientes scripts:

```bash
~/ProyectoCPR/build_all.sh
~/ProyectoCPR/run_all.sh lanzar_pure_pursuit.launch true
```
Puedes cambiar `true` por `false` para no lanzar RViz.


