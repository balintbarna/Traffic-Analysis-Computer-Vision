# Getting started

Download video:
```
cd src/traffic_analysis_from_drones/data
make get_data
```

Check the for the correct environment in Pipfile.

Get back to the main directory (`traffic_analysis_shared`)
and fetch the submodule with the image stabilizer.
```
cd ../../..
git submodule init
git submodule update
```
Build and launch.
```
source /opt/ros/melodic/setup.bash
catkin build
source devel/setup.bash
roslaunch traffic_analysis_from_drones track_traffic.launch
```

