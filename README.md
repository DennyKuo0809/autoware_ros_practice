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
* (If needed) Clear workspace
```sh=
cd autoware_ros_practice
rm -rf log/ install/ build/
```

#### Usage
* Run autoware planning simulation
    * refer to [autoware documentation](https://autowarefoundation.github.io/autoware-documentation/galactic/tutorials/ad-hoc-simulation/planning-simulation/)
```sh=
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```
* Click panel to set 2D Estimator
* Run our goal_pose_setter
    * Setup
    ```sh=
    cd autoware_ros_practice
    . install/setup.bash
    ```
    * Run the node with parameters
        * <span style="color:yellow">(For demo)</span> replace the `[demo parameter file]` below with `demo_parameters/goal_pose.yaml`
        * Follow this [link](https://roboticsbackend.com/ros2-yaml-params/) to write your own parameter file in yaml format
    ```sh=
    ros2 run cpp_set_goal_pose goal_pose_setter --ros-args --params-file [demo parameter file]
    ```