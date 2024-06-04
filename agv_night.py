#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
import socket
import threading

def move_to_goal(x, y):
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not ac.wait_for_server(rospy.Duration(5)):
        rospy.logerr("Action server not available!")
        return False

    rospy.loginfo("Connected to move_base action server")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    rospy.loginfo("Sending goal: ({}, {})".format(x, y))
    ac.send_goal(goal)
    ac.wait_for_result()

    state = ac.get_state()
    rospy.loginfo("Current state: {}".format(state))

    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("목표 지점에 도착했습니다!")
        return True
    else:
        rospy.logerr("목표 지점 도달 실패... 상태: {}".format(state))
        return False

def get_goal_coordinates(case_number):
    if case_number == 1:
        rospy.loginfo("Returning coordinates for case 1")
        return (1.0, 1.0)
    elif case_number == 2:
        rospy.loginfo("Returning coordinates for case 2")
        return (1.0, 1.2)
    elif case_number == 3:
        rospy.loginfo("Returning coordinates for case 3")
        return (1.0, 1.4)
    elif case_number == 4:
        rospy.loginfo("Returning coordinates for case 3")
        return (-0.3, -0.6)
    else:
        rospy.loginfo("유효하지 않은 케이스 번호입니다.")
        return (None, None)

def handle_client(client_socket):
    case_number = 1  # 초기 케이스 번호 설정

    def wait_for_message():
        data = client_socket.recv(1024).decode().strip()  # 공백 제거
        rospy.loginfo("Received data: {}".format(data))
        return data

    while case_number <= 4:  # 3번 케이스까지 이동
        rospy.loginfo("다음 'dark' 메시지를 기다립니다...")

        message_thread = threading.Thread(target=wait_for_message)
        message_thread.start()
        message_thread.join(timeout=10)  # 10초 동안 메시지를 기다립니다.

        if not message_thread.is_alive():  # 메시지를 받지 못했을 경우
            data = "dark"  # 기본값을 "dark"로 설정하여 자동 진행
        else:
            data = wait_for_message()

        if data == "dark":
            rospy.loginfo("클라이언트로부터 'dark' 메시지를 받았습니다. 케이스 번호: {}".format(case_number))
            goal_x, goal_y = get_goal_coordinates(case_number)
            if goal_x is not None and goal_y is not None:
                rospy.loginfo("이동할 좌표: ({}, {})".format(goal_x, goal_y))
                if move_to_goal(goal_x, goal_y):
                    rospy.loginfo("케이스 번호 {} 지점에 도착했습니다. 다음 좌표로 이동합니다.".format(case_number))
                    case_number += 1
                else:
                    rospy.loginfo("목표 지점 도달 실패. 다시 시도합니다.")
            else:
                rospy.loginfo("유효하지 않은 좌표입니다.")
                break
        else:
            rospy.loginfo("알 수 없는 메시지: {}".format(data))
            move_to_goal(-0.3, -0.6)
            break 

if __name__ == '__main__':
    try:
        rospy.init_node('simple_navigation', anonymous=True)
        rospy.loginfo("간단한 내비게이션을 시작합니다...")

        HOST = '0.0.0.0'
        PORT = 12345

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        rospy.loginfo("AGV 대기 중...")

        while not rospy.is_shutdown():
            client_socket, addr = server_socket.accept()
            rospy.loginfo("연결됨: %s", addr)
            handle_client(client_socket)
            client_socket.close()

        server_socket.close()

    except rospy.ROSInterruptException:
        rospy.loginfo("내비게이션 테스트 종료.")
