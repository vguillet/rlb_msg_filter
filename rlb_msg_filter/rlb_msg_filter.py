
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from rlb_utils.msg import Goal, TeamComm, CommsState

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import pandas as pd
import math
import json

from rlb_config.robot_parameters import *

# ================================================================================= Main



class RLB_msg_filter(Node):
    def __init__(self):
        # -> Initialise inherited classes
        Node.__init__(self, 'msg_filter')

        # -> Setup robot ID
        self.declare_parameter('robot_id', 'Turtle')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # -> Setup robot comms state matrix
        self.comms_state_matrix = None

        # ----------------------------------- Comms matrix state subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.comms_state_matrix_pub = self.create_subscription(
            msg_type=CommsState,
            topic="/comms_state_matrix",
            callback=self.comms_state_matrix_callback,
            qos_profile=qos
        )

        # ----------------------------------- Team comms subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.team_comms_subscriber = self.create_subscription(
            msg_type=TeamComm,
            topic="/team_comms",
            callback=self.filter_msg,
            qos_profile=qos
        )

        # ----------------------------------- Filter output publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.output_publisher = self.create_publisher(
            msg_type=TeamComm,
            topic=f"/{self.robot_id}/sim/filter_output",
            qos_profile=qos 
        )

    def comms_state_matrix_callback(self, msg):
        # -> Load msg json
        comms_state = json.loads(msg.comms_states)
        
        # -> Load comms states matrix df
        matrix = pd.read_json(comms_state["comms_state_matrix"])

        if not matrix.empty:
            self.comms_state_matrix = matrix
        
    def filter_msg(self, msg):
        # -> Check msg target
        if msg.target not in [self.robot_id, "all"] or msg.source == self.robot_id:
            return

        # -> Check comms state
        if self.comms_state_matrix is not None:
            try:
                comms_state = self.comms_state_matrix[msg.source][self.robot_id]

                if comms_state:
                    self.output_publisher.publish(msg)
            except:
                return

def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = RLB_msg_filter()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()