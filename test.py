import rospy
from mavros_msgs.msg import OverrideRCIn
import time

def main():
    rospy.init_node('forward_and_turn')
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    rate = rospy.Rate(100)  # ループの実行頻度（10 Hz）

    # サーボチャンネルの設定
    servo_channel_forward = 1

    servo_channel_reverse = 3
    forward_value = 1800  #, 前進のサーボ値
    reverse_value = 1500  # 中立のサーボ値（停止）
    turn_value = 1800     # 右に90度回転するサーボ値

    # 前進
    start_time = time.time()
    while time.time() - start_time < 5.0:
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = 1800
        
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        rate.sleep()
        print("a")

    # 停止
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    rate.sleep()

    # 右に90度回転
    start_angle = 0.0
    while abs(start_angle - 90.0) > 5.0:  # 目標角度までの誤差が5度未満になるまで
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = 1200
        msg.channels[servo_channel_reverse - 1] = reverse_value
        pub.publish(msg)
        rate.sleep()
        # 現在の角度を更新する仮定
        start_angle += 0.35 # 例: 毎秒0.1度回転したと仮定
        if start_angle > 360.0:  # 360度を超えた場合、リセット
            start_angle = 0.0

    # 停止
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    rate.sleep()
    print("a")
    start_time = time.time()
    while time.time() - start_time < 5.0:
        msg = OverrideRCIn()
        msg.channels[servo_channel_forward - 1] = 1500
        
        msg.channels[servo_channel_reverse - 1] = 1800
        pub.publish(msg)
        rate.sleep()
        print("b")

    # 停止
    msg = OverrideRCIn()
    msg.channels[servo_channel_forward - 1] = reverse_value
    msg.channels[servo_channel_reverse - 1] = reverse_value
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass