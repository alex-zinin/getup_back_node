This repo contains nao nodes for getup


1) Move to your ROS2 workspace to src:
```
cd dev_ws/src
```
2) Download this node:
```
git clone https://github.com/alex-zinin/getup_back_node.git
```
3) Make rosdep install in your dev_ws directory: 

```
cd ..
rosdep install -i --from-path src --rosdistro galactic -y
```
4) Make colcon build:
```
colcon build --packages-select getup_back_node
```
5) To run this node:
```
. install/setup.bash
ros2 run getup_back_node getup_node
```
