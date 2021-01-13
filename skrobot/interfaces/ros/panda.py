import control_msgs.msg
import actionlib
import rospy
import franka_gripper.msg

from .base import ROSRobotInterfaceBase


class PandaROSRobotInterface(ROSRobotInterfaceBase):

    def __init__(self, *args, **kwargs):
        super(PandaROSRobotInterface, self).__init__(*args, **kwargs)

        self.gripper_stop = actionlib.SimpleActionClient(
            'franka_gripper/stop',
            franka_gripper.msg.StopAction)
        self.gripper_stop.wait_for_server()

        self.gripper_move = actionlib.SimpleActionClient(
            'franka_gripper/move',
            franka_gripper.msg.MoveAction)
        self.gripper_move.wait_for_server()

    @property
    def rarm_controller(self):
        return dict(
            controller_action='position_joint_trajectory_controller/follow_joint_trajectory',  # NOQA
            controller_state='position_joint_trajectory_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[j.name for j in self.robot.rarm.joint_list],
        )

    def default_controller(self):
        return [self.rarm_controller]

    def grasp(self, width=0, speed=0.08):
        goal = franka_gripper.msg.MoveGoal(width=0, speed=speed)
        self.gripper_move.send_goal(goal)  # does not wait

    def ungrasp(self, speed=0.08):
        goal = franka_gripper.msg.StopGoal()
        self.gripper_stop.send_goal_and_wait(goal)

        goal = franka_gripper.msg.MoveGoal(width=0.08, speed=speed)
        self.gripper_move.send_goal_and_wait(goal)
