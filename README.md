


```
source ~/tiago_public_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
roslaunch tiago_iaslab_simulation navigation.launch
roslaunch tiago_iaslab_simulation apriltag.launch
```

if you want to run it in DEBUG mode for blue object:
```
roslaunch assignment2 run.launch

```
else:
```
roslaunch assignment2 run.launch DEBUG:=False

```
