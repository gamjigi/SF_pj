import socket
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion

def move_to_goal(x, y):
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
    ac.send_goal(goal)
    ac.wait_for_result()
    if ac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("목표 지점에 도착했습니다!")
        return True
    else:
        rospy.loginfo("목표 지점 도달 실패...")
        return False

def get_goal_coordinates(case_number):
    if case_number == 401:
        return (4.0, 1.0)
    elif case_number == 402:
        return (4.0, 2.0)
    elif case_number == 403:
        return (4.0, 3.0)
    else:
        rospy.loginfo("유효하지 않은 케이스 번호입니다.")
        return (None, None)

def handle_client(client_socket):
    while True:
        data = client_socket.recv(1024).decode()
        if data in ["401", "402", "403"]:
            rospy.loginfo(f"{data}을 받았습니다. 좌표 이동을 시작합니다.")
            try:
                goal_x, goal_y = get_goal_coordinates(int(data))
                if goal_x is not None and goal_y is not None:
                    move_to_goal(goal_x, goal_y)
                else:
                    rospy.loginfo("유효하지 않은 좌표입니다. 다시 시도하세요.")
            except ValueError:
                rospy.loginfo("잘못된 입력입니다. 숫자를 입력하세요.")
        else:
            rospy.loginfo("유효하지 않은 데이터입니다.")

def main():
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

if __name__ == '__main__':
    main()