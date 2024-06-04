#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def move_to_pose(target_x, target_y, target_z, target_orientation_w=1.0, target_orientation_x=0.0, target_orientation_y=0.0, target_orientation_z=0.0, end_effector_link="link6"):
    # 초기화
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm_group")  # Move Group 이름 확인 필요
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    rospy.loginfo("Starting move_to_pose function")

    # 현재 위치 출력
    current_pose = group.get_current_pose().pose
    rospy.loginfo("Current pose:")
    rospy.loginfo("Position: x=%f, y=%f, z=%f" % (current_pose.position.x, current_pose.position.y, current_pose.position.z))
    rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f" % (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w))

    # 목표 위치 설정
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation.w = target_orientation_w
    target_pose.orientation.x = target_orientation_x
    target_pose.orientation.y = target_orientation_y
    target_pose.orientation.z = target_orientation_z
    target_pose.position.x = target_x  
    target_pose.position.y = target_y
    target_pose.position.z = target_z  

    # 목표 링크 설정
    group.set_end_effector_link(end_effector_link)
    group.set_pose_target(target_pose, end_effector_link)

    # 목표 위치 출력
    rospy.loginfo("Target pose:")
    rospy.loginfo("Position: x=%f, y=%f, z=%f" % (target_pose.position.x, target_pose.position.y, target_pose.position.z))
    rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f" % (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w))
    rospy.loginfo("End effector link: %s" % end_effector_link)

    # 플래닝 시간 설정
    group.set_planning_time(30)  # 플래닝 시간을 30초로 늘림

    # 플래닝 및 실행
    rospy.loginfo("Starting planning")
    plan = group.go(wait=True)

    if plan:
        rospy.loginfo("Planning successful")
    else:
        rospy.loginfo("Planning failed")

    rospy.loginfo("Stopping the robot")
    group.stop()
    rospy.loginfo("Clearing pose targets")
    group.clear_pose_targets()

    # 종료
    rospy.loginfo("Shutting down MoveIt Commander")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # 원하는 목표 위치와 자세 설정
        move_to_pose(0.2, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, end_effector_link="link6")        
    except rospy.ROSInterruptException:
        pass
