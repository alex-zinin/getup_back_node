# Getup Node


### Description

This repo contains nao nodes for getup




This node monitors the gyroscope sensor and, in case of a fall on the back, starts the execution of the rising poses. It is worth noting that the poses are performed sequentially one after another. The values of the angles of these poses are stored as an array in the python script of this node, which of course is not very good and requires refinement with the removal of poses in a separate file. It is also very important to note that when developing the stand-up position, it was assumed that the robot has 23 joints and 3 more are equal to zero, the sequence of which is strictly fixed, following the NAO documentation [GitHub Pages](https://nao-interfaces-docs.readthedocs.io/en/latest/joints.html):


```
uint8 HEADYAW=0
uint8 HEADPITCH=1
uint8 LSHOULDERPITCH=2
uint8 LSHOULDERROLL=3
uint8 LELBOWYAW=4
uint8 LELBOWROLL=5
uint8 LWRISTYAW=6
uint8 LHIPYAWPITCH=7
uint8 LHIPROLL=8
uint8 LHIPPITCH=9
uint8 LKNEEPITCH=10
uint8 LANKLEPITCH=11
uint8 LANKLEROLL=12
uint8 RHIPROLL=13
uint8 RHIPPITCH=14
uint8 RKNEEPITCH=15
uint8 RANKLEPITCH=16
uint8 RANKLEROLL=17
uint8 RSHOULDERPITCH=18
uint8 RSHOULDERROLL=19
uint8 RELBOWYAW=20
uint8 RELBOWROLL=21
uint8 RWRISTYAW=22
uint8 LHAND=23
uint8 RHAND=24
uint8 NUMJOINTS=25
```

If we talk about the triggering of the node, then we can say that it is done by tracking the topic / sensors / gyroscope, in which information about the current position of the robot in space is published. Accordingly, in the case of falling on your back, the value along the axis of the gyroscope becomes small and the need to get up is triggered., launching the publication of messages with the values of the required angles of the joints in the topic /effectors/joint_positions.



### Running this node

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
*Make sure that nao_lola node is running:

```
source /opt/ros/galactic/setup.bash
ros2 run nao_lola nao_lola
```
