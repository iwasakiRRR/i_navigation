#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import tf
import time
import sys
import std_srvs.srv
import os
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from kobuki_msgs.msg import DigitalInputEvent
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import subprocess##

ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
vel = Twist()
goal = MoveBaseGoal()
waypoint_number = 0
state_change_number = 999
digital_in = [False,True, True, True]#緊急停止スイッチ

class State:
	Start = 0
	GoWp = 1
	Follow= 4
	RArena= 5
	first_num = 0
	
class Robot:
	#waypoint = [[-1.18,-7.08,0],[-0.6,-6.12,0],[-1.27,-3.94,0]]#iwasaki_test_mymap3
	waypoint = [[4.23,-0.513,0],[5.41,-0.341,0],[5.3,-4.27,0],[5.18,-12.4,0],[5.94,-16.4,0],[5.17,-11.9,0],[5.2,-4.52,0],[5.03,-0.503,0],[2.19,-0.281,0]]#iwasaki_test_mymap4

	x = 0.1
	y = 0.1
	Human_flg = False
	nav_time_cnt = 0
	laser_dists = range(1080)
	speech_str = "False"
	follow_finish = "none"
	first_human_flg = 0#最初に人を見つけたとき１になる
	wp4_through_flg = 0
	wp5_through_flg = 0
	wpnum_wp2 = 1
	wpnum_wp3 = 3
	say_stop_cnt = 0
		
class I_NAV2:#e_navigation2 main class
	def __init__(self):
		self.clear_costmap_cnt = 0
		self.clear_costmap = rospy.ServiceProxy('move_base/clear_costmaps',std_srvs.srv.Empty)

	def DigitalInputEventCallback(self,data):
		global digital_in
		digital_in = data.values

	rospy.Subscriber('/mobile_base/events/digital_input', DigitalInputEvent, DigitalInputEventCallback)

	#---------------Publish,Subscribe
	def BaseCallback(Message):#自己座標を取得
		Robot.x = Message.pose.pose.position.x
		Robot.y = Message.pose.pose.position.y
	def Laser_Callback(laser_scan):
		Robot.laser_dists[540] = laser_scan.ranges[540]
	def HumanCallback(Message):#障害物情報を受信
		Robot.Human_flg = Message.data
	def SpeechCallback(Message):#音声認識内容受け取り
		Robot.speech_str = Message.data
	ac_pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size = 10)#アクションサーバに指示送る用
	laser_sub = rospy.Subscriber('/scan', LaserScan, Laser_Callback)#ライダー情報取得
	odom_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, BaseCallback)#オドメトリ情報取得
	find_human_sub = rospy.Subscriber('/find_human',String,HumanCallback)#e_followから、人検出でTrueが送られてくる
	voice_sub = rospy.Subscriber('/voice_recog',String,SpeechCallback)#音声認識内容受け取り
	follow_pub = rospy.Publisher('/follow_human',String,queue_size = 10)#e_followプログラム追従指令用
	#---------------------------------
	#アクションサーバにゴール座標送信
	def Navigate_Waypoint(self,WpNum):
                if ac.wait_for_server(rospy.Duration(5)) == 1:
                        print "Wait For Action Client Rising Up!"
                goal.target_pose.header.frame_id = 'map'         # 地図座標系
                goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
                goal.target_pose.pose.position.x =  Robot.waypoint[WpNum][0]
                goal.target_pose.pose.position.y =  Robot.waypoint[WpNum][1]
                q = tf.transformations.quaternion_from_euler(0, 0, Robot.waypoint[WpNum][1])
                goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[1],q[3])
                ac.send_goal(goal);

	def Door_Open_Detection(self):     
		#障害物検知でobst_flgを立てる------------------------
		if Robot.laser_dists[540] < 0.5 or Robot.laser_dists[540] > 50:
			self.obst_flg = 1
		else:
			sefl.odst_flag = 0;
		if self.obst_flg == 0:#障害物フラグOff
			print "State change to going waypoint1"
			return State.GoWp##hlorhgorhfoughhgorhfrhfp
		elif self.obst_flg == 1:
			return State.Start

	def Navigate_Wp(self):
		if State.first_num == 0:
			self.Navigate_Waypoint(waypoint_number)
			State.first_num = 1
			self.clear_costmap_cnt += 1
		if self.clear_costmap_cnt >= 500:
			self.clear_costmap()
			self.clear_costmap_cnt = 0
			State.first_num = 0
		dist_to_goal = math.hypot(Robot.x-goal.target_pose.pose.position.x,Robot.y-goal.target_pose.pose.position.y)#ゴールとの距離

		if dist_to_goal < 0.5:#ゴールに到着(0.5m以内)
			ac.cancel_goal();
			cmd3 = "/usr/bin/picospeaker %s" % "I reach to waypoint one"
			##subprocess.call(cmd3.strip().split(" "))
			State.first_num = 0
			if state_change_number == waypoint_number:
				waypoint_number += 1
				return State.Follow
			waypoint_number += 1
			return State.GoWp
		else:
			print "Now Waiting!"
		return State.GoWp
if __name__ == '__main__':
	rospy.init_node('e_navigation2')#nodeの初期化
	operate = State.Start
	i_nav2 = I_NAV2()
	cold_start = 1
	while(cold_start):#1秒待機
		cmd3 = "/usr/bin/picospeaker %s" % "I start waypoint navigation"
		##subprocess.call(cmd3.strip().split(" "))
		cold_start -= 1
		time.sleep(3.0)
	while not rospy.is_shutdown():#シャットダウンした時に抜ける
		if digital_in[0] == True:
			vel.linear.x = 0.0
			vel.angular.z = 0.0
			ac_pub.publish(vel)
			print "緊急停止スイッチ作動！！"
			rate = rospy.Rate(5000)
			rate.sleep()
			continue
		if waypoint_number == len(Robot.waypoint) + 1:
			operate = State.Finish
			print "Navigation finished"
		if operate == State.Start:
			operate = i_nav2.Door_Open_Detection()
		elif operate == State.GoWp:
			operate = i_nav2.Navigate_Wp()
		elif operate == State.Follow:
			operate = i_nav2.Follow_Human()
		elif operate == State.RArena:
			operate = i_nav2.Return_Arena()
				
		rate = rospy.Rate(5)
		rate.sleep()
