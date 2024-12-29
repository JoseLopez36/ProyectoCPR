@echo off
set CONTAINER_NAME=airsim_container

:: Verificar si el contenedor está en ejecución
docker ps -q --filter "name=%CONTAINER_NAME%" >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo El contenedor %CONTAINER_NAME% no está en ejecución.
    pause
    exit /b
)

:: Adjuntar una nueva terminal al contenedor
docker exec -it %CONTAINER_NAME% bash