@echo off
set CONTAINER_NAME=airsim_container

docker run --rm -it --net=host ^
-v %CPR_PATH%:/home/testuser/ProyectoCPR ^
--name %CONTAINER_NAME% ^
airsim-ros:latest bash