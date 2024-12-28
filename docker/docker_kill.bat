@echo off
set CONTAINER_NAME=airsim_container

FOR /F "tokens=*" %%i IN ('docker ps -q -f "name=%CONTAINER_NAME%"') DO (
    set RUNNING_CONTAINER=%%i
)

if defined RUNNING_CONTAINER (
    echo Deteniendo el contenedor: %CONTAINER_NAME%
    docker stop %RUNNING_CONTAINER%
    echo Eliminando el contenedor: %CONTAINER_NAME%
    docker rm %RUNNING_CONTAINER%
) else (
    echo No se encontro ningun contenedor en ejecucion con el nombre: %CONTAINER_NAME%
)