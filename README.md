
# myMAV

Gazebo simulation of an UAV with a mounted FT sensor for peg-in-hole assembly

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

To be able to launch myMAV, following packages are required:

 > **Note** Follow install instructions from: https://github.com/ethz-asl/rotors_simulator


### Installing
In your catkin workspace:

 ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/BevandaFER/mymav.git
 ```

Build your workspace:
```
$ cd ~/catkin_ws/
$ catkin init  # If you haven't done this before.
$ catkin build
```

### Launching the simulator
For height control
```

$ roslaunch mymav_gazebo mymav_height.launch
```
For force control
```

$ roslaunch mymav_gazebo mymav_force.launch
```

For hybrid control
```

$ roslaunch mymav_gazebo mymav_hybrid.launch
```


## Authors

* **Petar Bevanda** - *Initial work* - [myMAV](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## Acknowledgments

* Bachelor thesis 

