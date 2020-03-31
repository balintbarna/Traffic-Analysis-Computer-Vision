# Getting started

To get started with this development environment for the 
traffic analysis miniproject, do the following:

Enter the directory `traffic_analysis_shared`.

Download an example dataset.
```
cd src/traffic_analysis_from_drones/data
pipenv install
make get_data
```

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

