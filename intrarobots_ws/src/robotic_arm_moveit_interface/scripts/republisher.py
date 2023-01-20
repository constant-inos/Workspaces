import rospy
from sensor_msgs.msg import PointCloud2


POINT_CLOUD_TOPIC1 = '/camera/depth/points'
POINT_CLOUD_TOPIC2 = '/camera2/depth/points'

point_cloud_subscriber1 = rospy.Subscriber(POINT_CLOUD_TOPIC1, PointCloud2, get_ptc_1)
point_cloud_subscriber2 = rospy.Subscriber(POINT_CLOUD_TOPIC2, PointCloud2, get_ptc_2)

point_cloud_publisher = rospy.Publisher("/"+ROBOT_NAME+"/go_to_pose_command",GoToPoseCommand,queue_size=1)
