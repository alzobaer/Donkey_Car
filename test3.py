#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import OverrideRCIn

def send_rc_override():
    """
    RCオーバーライド信号をPixhawkに送信するプログラム
    """
    rospy.init_node('rc_override_node', anonymous=True)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # オーバーライド信号の初期化（すべてのチャンネルをニュートラル値に設定）
    rc_msg = OverrideRCIn()
    # rc_msg.channels = [1500, 1500, 1500, 1500, 0, 0, 0, 0]  # [Roll, Pitch, Throttle, Yaw, Aux1, Aux2, Aux3, Aux4]

    rospy.loginfo("Sending RC override signals...")

    # 10秒間、RCオーバーライド信号を送信
    for i in range(100):
        # サンプル：スロットルを少し上げる (チャンネル3: 1600)
        rc_msg.channels[2] = 1800  # Throttle

        # サンプル：Yaw（チャンネル4）を回転させる場合
        # rc_msg.channels[3] = 1600  # Clockwise rotation

        pub.publish(rc_msg)
        rate.sleep()

    # オーバーライドを解除（全チャンネルを`0`に設定）
    # rc_msg.channels = [0] * 8
    pub.publish(rc_msg)
    rospy.loginfo("RC override stopped, channels reset.")

if __name__ == "__main__":
    try:
        send_rc_override()
    except rospy.ROSInterruptException:
        pass
