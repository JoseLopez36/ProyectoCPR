@echo off

set IP_RVIZ=172.28.192.1

set CONTAINER_NAME=airsim_container

docker run --rm -it --net=host ^
-v %CPR_PATH%:/home/testuser/ProyectoCPR ^
--name %CONTAINER_NAME% ^
-e DISPLAY=%IP_RVIZ% ^
airsim-ros:latest bash