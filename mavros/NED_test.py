import rospy
import tf

from mavros_msgs.msg import PositionTarget
class MavrosTestCommon(unittest.TestCase):
    def __init__(self, *args):
        super(MavrosTestCommon, self).__init__(*args)
        self.transformer = tf.TransformListener()

    # ...

    def assertWaypointsEqual(self, expected_waypoints, actual_waypoints):
        expected_points = [self.convertROSToNED(pos) for pos in expected_waypoints]
        actual_points = [self.convertROSToNED(pos) for pos in actual_waypoints]
        self.assertListEqual(expected_points, actual_points)

class DroneController:
    def __init__(self):
        self.transformer = tf.TransformListener()
        self.target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    def convertROSToNED(self, position):
        try:
            self.transformer.waitForTransform("map", "ned", rospy.Time(), rospy.Duration(2.0))
            (trans, rot) = self.transformer.lookupTransform("map", "ned", rospy.Time(0))
            transformed_pos = self.transformer.transformPoint("ned", position)
            return transformed_pos.point
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.fail("Failed to convert ROS frame to NED frame")

    def assertWaypointsEqual(self, expected_waypoints, actual_waypoints):
        expected_points = [self.convertROSToNED(pos) for pos in expected_waypoints]
        actual_points = [self.convertROSToNED(pos) for pos in actual_waypoints]
        assert len(expected_points) == len(actual_points), "Mismatch in the number of waypoints"
        for expected, actual in zip(expected_points, actual_points):
            assert expected == actual, "Waypoint mismatch: expected={}, actual={}".format(expected, actual)

    def navigateToPosition(self, position):
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]
        self.target_pub.publish(msg)

# Example usage
rospy.init_node('drone_controller_node')

controller = DroneController()

# Example waypoints in ROS frame
ros_waypoints = [(1, 2, 3), (4, 5, 6), (7, 8, 9)]

# Convert and assert waypoints in NED frame
controller.assertWaypointsEqual(ros_waypoints, ros_waypoints)

# Example target position in ROS frame
ros_target_position = (10, 11, 12)

# Convert and navigate to target position in NED frame
controller.navigateToPosition(ros_target_position)

rospy.spin()