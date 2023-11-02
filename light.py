#!/usr/bin/env python3


""" 
    @Strix Elixel 
    @program light_example
    @description Simple Example to show how to make request to embedded system via ROS2 Service Call via CAN Driver
"""


import rclpy
from rclpy.node import Node
from can_client import Can_Client


########################################
BLINK_PERIOD            = .200 #seconds
LIGHT_NODE_NAME         = 'light_node'
LIGHT_OFF               = 0
########################################


class LightNode(Node):
    
    light_state = LIGHT_OFF

    def __init__(self):
        """ 
            We will init our can_client to use it
            Then we have to enable the light and run the blink logic on a timer
        """
        super().__init__(LIGHT_NODE_NAME)
        self.can_client = Can_Client()
        self.can_client.enableLight()
        self.timer = self.create_timer(BLINK_PERIOD, self.blink)
    

    def blink(self):
        self.can_client.turnOnLight() if self.light_state == LIGHT_OFF else self.can_client.turnOffLight 
        self.light_state = 1 - self.light_state # Change Light State 


def main(args=None):
    """ 
        Running node simply involves instantiating class and spinning 
    """
    rclpy.init(args=args)
    light_node = LightNode()
    rclpy.spin(light_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()