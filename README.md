The software is developed and tested with ROS Noetic and Ubuntu 20.04 LTS

Clone the repository to the src folder of your workspace:
```
cd catkin_ws/src
git clone https://github.com/NMMI/aerial-alter-ego
```
then
```
cd ..
catkin_make
```

Usage, the command to launch
The command to launch the head and arm control:
```
roslaunch global_control arm_teleoperation.launch
```
The command to launch the head and arm control in simulation:
```
roslaunch global_control arm_teleoperation_sim.launch
```


Compile and run ground station zed_oculus_drone (Windows OS visual studio 2015) 

Istitutions
Istituto Italiano di Tecnologia, via Morego, 30, 16163 Genova, Italia

Centro di Ricerca E. Piaggio e Dipartimento di Ingegneria dell’Informazione, Universita di Pisa, Pisa, Italia
