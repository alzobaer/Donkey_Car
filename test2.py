from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu
import rospy
from math import degrees
from tf.transformations import euler_from_quaternion

global global_yaw_degrees
global_yaw_degrees = 0

def imu_callback(data):
    global global_yaw_degrees
    # 地磁気からヨー角を計算
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
 
    # ラジアンから度に変換
    global_yaw_degrees = degrees(yaw)
    global_yaw_degrees += 180
    print(global_yaw_degrees)
pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
imu_pub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)

ori_sub = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)

print(global_yaw_degrees)
print('done')