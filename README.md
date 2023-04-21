# autoware ros practice

## cpp_set_goal_pose
A talker node written in c++, to set the goal pose of vehicle in autoware.
#### Prerequisites
* Ubuntu20.04
* Autoware.universe galactic version with ros2 galactic
#### Build
* Download
```sh=
git clone https://github.com/DennyKuo0809/autoware_ros_practice.git
cd autoware_ros_practice
```
* Install dependency
```sh=
rosdep update --rosdistro=galactic
rosdep install -i --from-path src --rosdistro galactic -y
```

* Build workspace
```sh=
colcon build --packages-select cpp_set_goal_pose
```

#### Usage
* Run autoware planning simulation
    * refer to [autoware documentation](https://autowarefoundation.github.io/autoware-documentation/galactic/tutorials/ad-hoc-simulation/planning-simulation/)
```sh=
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

* Click panel to set 2D Estimator
* Run our goal_pose_setter
    * setup
    ```sh=
    cd autoware_ros_practice
    . install/setup.bash
    ```
    * run the node
    ```sh=
    ros2 run cpp_set_goal_pose goal_pose_setter
    ```