# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nao_sensor_msgs.msg import Gyroscope
from std_msgs.msg import String
import time

from nao_command_msgs.msg import JointPositions

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(JointPositions, '/effectors/joint_positions', 10)
        self.subscription = self.create_subscription(
            Gyroscope,
            '/sensors/gyroscope',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if abs(msg.y) <= 2.4233590134592475e-9:
            self.get_logger().info('I heard: "%s"' % msg)
            time.sleep(10.0)    # Pause 5.5 seconds
            self.get_logger().info('time')

            poses = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -4, 0, 0, 0, 0, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 115, 74, 0, -80, -103, 0, 15, 23, -4, 50, 0, -15, 23, -4, 50, 0, 115, -75, 0, 80, 102, 0, 0],
             [0, 0, 115, 74, 0, -80, -103, 0, 15, 23, 20, 50, 0, -15, 23, 20, 50, 0, 115, -75, 0, 80, 102, 0, 0],
             [0, 0, 115, 74, 0, -80, -103, 0, 15, 23, 55, 50, 0, -15, 23, 55, 50, 0, 115, -75, 0, 80, 102, 0, 0],
             [0, 0, 115, 10, 0, -80, -103, 0, 15, 0, 90, 50, 0, -15, 0, 90, 50, 0, 115, -10, 0, 80, 102, 0, 0],
             [0, 0, 115, 10, 0, -80, -103, 0, 15, -35, 90, 0, 0, -15, -35, 90, 0, 0, 115, -10, 0, 80, 102, 0, 0],
             [0, 0, 115, 10, 0, -75, -103, 0, 15, -35, 90, 0, 0, -15, -35, 90, 0, 0, 115, -10, 0, 75, 102, 0, 0],
             [0, 0, 115, 5, 0, -65, -103, 0, 15, 0, 75, 0, 0, -15, 0, 75, 0, 0, 115, -5, 0, 65, 102, 0, 0],
             [0, 0, 117, 5, 0, -65, -103, 0, 15, 0, 100, 0, 0, -15, 0, 100, 0, 0, 117, -5, 0, 65, 102, 0, 0],
             [0, 0, 117, 5, 0, -65, -103, 0, 15, 0, 100, 50, 0, -15, 0, 100, 50, 0, 117, -5, 0, 65, 102, 0, 0],
             [0, 0, 115, 5, 0, -65, -103, 0, 15, 0, 75, 50, 0, -15, 0, 75, 50, 0, 115, -5, 0, 65, 102, 0, 0],
             [0, 0, 117, 7, 0, -76, -103, 0, 15, 0, 75, 50, 0, -15, 0, 75, 50, 0, 117, -7, 0, 78, 102, 0, 0],
             [0, 0, 117, 7, 0, -76, -103, 0, 15, -35, 75, 50, 0, -15, -35, 75, 50, 0, 117, -7, 0, 78, 102, 0, 0],
             [0, 0, 117, 7, 0, -76, -103, 0, 15, -45, 75, 50, 0, -15, -45, 75, 50, 0, 117, -7, 0, 78, 102, 0, 0],
             [0, 0, 117, 7, 0, -76, -103, 0, 15, -45, 35, 50, 0, -15, -45, 35, 50, 0, 117, -7, 0, 78, 102, 0, 0],
             [0, 0, 115, 7, 0, -76, -103, 0, 0, -25, -4, 50, 0, 0, -25, -4, 50, 0, 115, -7, 0, 78, 102, 0, 0],
             [0, 0, 115, 7, 0, -76, -103, 0, 0, -15, -4, 50, 0, 0, -15, -4, 50, 0, 115, -7, 0, 78, 102, 0, 0],
             [0, 0, 115, 15, 0, -76, -103, 0, 0, -45, -4, 50, 0, 0, -45, -4, 50, 0, 115, -15, 0, 78, 102, 0, 0],
             [0, 0, 115, 20, 0, -76, -103, 0, 0, -45, -4, 50, 0, 0, -45, -4, 50, 0, 115, -20, 0, 78, 102, 0, 0],
             [0, 0, 115, 20, 0, -76, -103, 0, 0, -55, -4, 50, 0, 0, -55, -4, 50, 0, 115, -20, 0, 78, 102, 0, 0],
             [0, 0, 115, 35, 0, -65, -103, 0, 0, -55, -4, 50, 0, 0, -55, -4, 50, 0, 115, -35, 0, 65, 102, 0, 0],
             [0, 0, 115, 35, 0, -45, -103, 0, 0, -55, -4, 50, 0, 0, -55, -4, 50, 0, 115, -35, 0, 45, 102, 0, 0],
             [0, 0, 115, 35, 0, -45, -103, 0, 0, -87, -4, 50, 0, 0, -87, -4, 50, 0, 115, -35, 0, 45, 102, 0, 0]]

            for posa in poses:

            # for i in range(0, 10000000):
            #     pass_ += 1
                msg2 = JointPositions()
            #
                msg2.indexes = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
            # for j in range(0, 5):
            #     pos_ = []
            #     for i in range(1, 25):
            #         pos_.append(float(j))
            #
                msg2.positions = [float(el) for el in posa]  # [12.0, 0.0, 0.0, 12.0, 12.0, 0.0, 0.0, 0.0, 0.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #     # msg = {positions: [12.0, 0.0, 0.0, 12.0, 12.0, 0.0, 0.0, 0.0, 0.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            #     l_ = []
            #     for i in range(1, 25):
            #         l_.append(int(i))
            #     msg2.indexes = l_  # [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
            #
                self.publisher_.publish(msg2)

        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
