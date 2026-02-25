#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from mujoco_ros_msgs.msg import JointSet

class JointBridge:
    def __init__(self):
        rospy.init_node('joint_bridge_node')

        # 제어 모드 설정 (0: Position, 1: Torque)
        # 업로드된 mjros.cpp의 jointset_callback 로직에 따름 
        self.control_mode = rospy.get_param('~control_mode', 0) 

        # Publisher: MuJoCo가 구독하는 토픽 
        self.pub = rospy.Publisher('mujoco_ros_interface/joint_set', JointSet, queue_size=1)

        # Subscriber: GUI가 발행하는 토픽
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)
        
        rospy.loginfo("Joint Bridge Node Started (Mode: %d)", self.control_mode)

    def callback(self, msg):
        js_msg = JointSet()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.time = rospy.get_time() # 시뮬레이션 시간 동기화용 
        js_msg.MODE = self.control_mode

        # GUI에서 오는 각 조인트의 위치값을 JointSet 메시지에 매핑 
        # URDF의 조인트 순서와 MuJoCo 액추에이터 순서가 일치해야 함 [cite: 4, 6, 9, 13, 15]
        js_msg.position = list(msg.position)
        
        # 만약 토크 제어 모드라면 velocity나 effort를 매핑할 수 있음
        if self.control_mode == 1:
            js_msg.torque = list(msg.effort)

        self.pub.publish(js_msg)

if __name__ == '__main__':
    try:
        JointBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
