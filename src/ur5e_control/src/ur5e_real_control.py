import rospy
import geometry_msgs.msg
import moveit_commander
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg


def to_pose_msg(transform):
    """Convert a `Transform` object to a Pose message."""
    msg = geometry_msgs.msg.Pose()
    msg.position = to_point_msg(transform.translation)
    msg.orientation = to_quat_msg(transform.rotation)
    return msg

def to_point_msg(position):
    """Convert numpy array to a Point message."""
    msg = geometry_msgs.msg.Point()
    msg.x = position[0]
    msg.y = position[1]
    msg.z = position[2]
    return msg

def to_quat_msg(orientation):
    """Convert a `Rotation` object to a Quaternion message."""
    quat = orientation.as_quat()
    msg = geometry_msgs.msg.Quaternion()
    msg.x = quat[0]
    msg.y = quat[1]
    msg.z = quat[2]
    msg.w = quat[3]
    return msg


class Ur5eCommander(object):
    def __init__(self):
        self.name = "ur5e"
        self.connect_to_move_group()

    def connect_to_move_group(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.name)

    def goto_home(self):
        self.goto_joints([1.5708,-1.5708,-1.0472,-2.0944,1.5708,0])

    def goto_joints(self,joints,velocity_scaling=0.1,acceleration_scaling=0.1):
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
        self.move_group.set_joint_value_target(joints)
        plan = self.move_group.plan()[1]
        
        user_input = input("EXECUTE PLAN [y/n] : ")
        if user_input == "y":
            success = self.move_group.execute(plan, wait=True)
        else:
            print("ABORTED PLAN")
            success = False

        self.move_group.stop()
        return success
    
    def goto_pose(self,pose,velocity_scaling=0.1,acceleration_scaling=0.1):
        pose_msg = to_pose_msg(pose)
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
        self.move_group.set_pose_target(pose_msg)
        plan = self.move_group.plan()[1]

        user_input = input("EXECUTE PLAN [y/n] : ")
        if user_input == "y":
            success = self.move_group.execute(plan, wait=True)
        else:
            print("ABORTED PLAN")
            success = False
            
        self.move_group.clear_pose_targets()
        return success
    
class GripperController(object):
    def __init__(self):
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size=10)

        self.activate_gripper()

    def activate_gripper(self):
        # activate
        print("ACTIVATE GRIPPER")
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP  = 255
        self.command.rFR  = 150
        self.pub.publish(self.command)
        rospy.sleep(1.0)

    def gripper_control(self,width):
        # 0 < width < 140
        value = (-11/7)*width + 220
        print(value)
        self.command.rPR = int(value)

        self.pub.publish(self.command)
        rospy.sleep(1.0)