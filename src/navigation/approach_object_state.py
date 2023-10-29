import tf2_ros
import rospy
from aenum import Enum, NoAlias
from geometry_msgs.msg import Twist
from util.ros_utils import get_rosparam
from context import Context
from util.state_lib.state import State
from util.state_lib.state_machine import StateMachine
from abc import ABC, abstractmethod
from approach_target_base_state import ApproachTargetBaseState
from util.state_lib.state_publisher_server import StatePublisher
from search import SearchState
from state import DoneState


class ApproachObjectState(ApproachTargetBaseState):
    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def on_loop(self, context):
        if ApproachTargetBaseState.object_type is None:
            return SearchState
        if ApproachTargetBaseState.object_type == "object":
            object_pos = 0
            ApproachTargetBaseState.target_pos = object_pos
            if ApproachTargetBaseState.arrived:
                return DoneState

        return self

    def get_target_pos():
        pass

    def determine_next():
        pass
