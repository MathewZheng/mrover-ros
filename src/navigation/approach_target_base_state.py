import tf2_ros
import rospy
from aenum import Enum, NoAlias
from geometry_msgs.msg import Twist
from util.ros_utils import get_rosparam
from context import Context
from util.state_lib.state import State
from util.state_lib.state_machine import StateMachine
from abc import ABC, abstractmethod

from util.state_lib.state_publisher_server import StatePublisher

STOP_THRESH = get_rosparam("single_fiducial/stop_thresh", 0.7)
FIDUCIAL_STOP_THRESHOLD = get_rosparam("single_fiducial/fiducial_stop_threshold", 1.75)
DRIVE_FWD_THRESH = get_rosparam("waypoint/drive_fwd_thresh", 0.34)  # 20 degrees


class ApproachTargetBaseState(State):
    def __init__(self) -> None:
        super().__init__()
        self.object_type = None

    @abstractmethod
    def get_target_pos():
        None

    @abstractmethod
    def determine_next():
        None

    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def on_loop(self, context):
        target_pos = self.get_target_pos()
        cmd = context.rover.get_drive_command(target_pos, context.rover.get_pose(), STOP_THRESH, DRIVE_FWD_THRESH)
        self.determine_next()
        object_sub = rospy.Subscriber("object_type", str, self.obj_type_callback)

        # check object type, return state to go to
        if self.object_type == "post":
            return 
        # TODO: set next object state
        return object_sub  # return next object state

    def obj_type_callback(self, data):
        self.object_type = data




