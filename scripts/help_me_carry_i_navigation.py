#!/usr/bin/env python
# -*- coding: utf-8 -*

##!!follow→Determine the destination→Pick up paper bag→Navigation→Place it→Asking for help→guidance!!##
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
digital_in = [False,True,True,True]#緊急停止スイッチ

class TransitionNumber:
	state_transition_point_0 = 999#you can set waypoint numbers for state transition. 
	state_transition_point_1 = 999
	state_transition_point_2 = 999
	state_transition_point_3 = 999

class State:
	door = 0
	gowp = 1
	follow= 2
	branch_off = 3
	pick_up = 4
	place = 5
	ask = 6
	remember = 7
	wait = 8
	unknown = 999
	finish = 555
	first_num = 0
	
class Robot:
	#waypoint = [[-1.18,-7.08,0],[-0.6,-6.12,0],[-1.27,-3.94,0]]#iwasaki_test_mymap3
	#waypoint = [[3,0.119,0],[0.937,0.253,0],[-1.23,1.44,0],[1.04,-0.888,0]]#iwasaki_test_mymap4
	waypoint = [[4.26,-0.53,0],[5.26,-0.67,0],[5.21,-4.02,0],[5.25,-7.11,0],[5.22,-10.8,0],[6.04,-16.5,0]]#ryo_map
	car_waypoint = [0.0,0.0,0.0]#車の座標を格納するためのもの
	specific_waypoint = [0.0,0.0,0.0]#指定された座標が入るためのもの
	kitchen_waypoint = [2.95,-1.37,0]#キッチンの座標を格納(ryo_map)
	dining_waypoint = [-0.848,-0.221,0]#ダイニングの座標を格納(ryo_map)
	entrance_waypoint = [4.7,-0.694,0]#エントランスの座標を格納(ryo_map)
	#kitchen_waypoint = [2.65,-0.917,0]#キッチンの座標を格納(mymap4)
	#dining_waypoint = [-4.9,-0.04,0]#ダイニングの座標を格納(mymap4)
	#entrance_waypoint = [3.87,0.16,0]#エントランスの座標を格納(mymap4)
	x = 0.10
	y = 0.1
	Human_flg = False
	nav_time_cnt = 0
	laser_dists = range(1080)
	speech_str = "False"
	place_str = "False"
	follow_finish = "none"
	first_human_flg = 0#最初に人を見つけたとき１になる
	specific_navigation_flg = 0#
	guide_to_the_car_flg = 0
	wp4_through_flg = 0
	wp5_through_flg = 0
	wpnum_wp2 = 1
	wpnum_wp3 = 3
	say_stop_count = 0
	time_count = 0
		
class I_NAV:
	def __init__(self):
		self.clear_costmap_cnt = 0
		self.clear_costmap = rospy.ServiceProxy('move_base/clear_costmaps',std_srvs.srv.Empty)
		self.waypoint_number = 0
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
	def PlaceCallback(Message):##運ぶ場所を受け取り
		Robot.place_str = Message.data
	
	ac_pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size = 10)#アクションサーバに指示送る用
	laser_sub = rospy.Subscriber('/scan', LaserScan, Laser_Callback)#ライダー情報取得
	odom_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, BaseCallback)#オドメトリ情報取得
	find_human_sub = rospy.Subscriber('/find_human',String,HumanCallback)#e_followから、人検出でTrueが送られてくる
	voice_sub = rospy.Subscriber('/voice_recog',String,SpeechCallback)#音声認識内容受け取り
	place_sub = rospy.Subscriber('/answer',String,PlaceCallback)
	follow_pub = rospy.Publisher('/follow_human',String,queue_size = 10)#e_followプログラム追従指令用
	#---------------------------------
	#アクションサーバにゴール座標送信
	def DoorOpenDetection(self):     
	#障害物検知でobst_flgを立てる------------------------
		if Robot.laser_dists[540] < 0.5 or Robot.laser_dists[540] > 50:
			self.obst_flg = 1
		else:
			self.obst_flg = 0;
		if self.obst_flg == 0:#障害物フラグOff
			print "State change to going waypoint1"
			return State.gowp
		elif self.obst_flg == 1:
			return State.door
	
	def WaypointToServer(self,WpNum):
		if ac.wait_for_server(rospy.Duration(5)) == 1:
			print "Wait For Action Client Rising Up!"
			goal.target_pose.header.frame_id = 'map'         # 地図座標系
			goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
			goal.target_pose.pose.position.x =  Robot.waypoint[WpNum][0]
			goal.target_pose.pose.position.y =  Robot.waypoint[WpNum][1]
			q = tf.transformations.quaternion_from_euler(0, 0, Robot.waypoint[WpNum][1])
			goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[1],q[3])
			ac.send_goal(goal);
			
	def SpecificWaypointToServer(self):
		if ac.wait_for_server(rospy.Duration(5)) == 1:
			print "Wait For Action Client Rising Up!"
			goal.target_pose.header.frame_id = 'map'         # 地図座標系
			goal.target_pose.header.stamp = rospy.Time.now() # 現在時刻
			goal.target_pose.pose.position.x = Robot.specific_waypoint[0]
			goal.target_pose.pose.position.y = Robot.specific_waypoint[1]
			q = tf.transformations.quaternion_from_euler(0, 0, Robot.specific_waypoint[1])
			goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[1],q[3])
			ac.send_goal(goal);

	def NavigateWp(self):
		t_n = TransitionNumber()
		if State.first_num == 0:
			if Robot.specific_navigation_flg == 0:
				self.WaypointToServer(self.waypoint_number)
			elif Robot.guide_to_the_car_flg == 1:
				Robot.specific_waypoint[0] = Robot.car_waypoint[0]
				Robot.specific_waypoint[1] = Robot.car_waypoint[1]
				self.SpecificWaypointToServer()
			elif Robot.specific_navigation_flg == 1:
				self.SpecificWaypointToServer()
			State.first_num = 1
		self.clear_costmap_cnt += 1
		print "self.clear_costmap_cnt is %d" % self.clear_costmap_cnt
		if self.clear_costmap_cnt >= 200:#500
			self.clear_costmap()
			self.clear_costmap_cnt = 0
			State.first_num = 0
		dist_to_goal = math.hypot(Robot.x-goal.target_pose.pose.position.x,Robot.y-goal.target_pose.pose.position.y)#ゴールとの距離
		if dist_to_goal < 0.5:#ゴールに到着(0.5m以内)
			ac.cancel_goal();
			#cmd3 = "/usr/bin/picospeaker %s" % "I reach to waypoint one"
			##subprocess.call(cmd3.strip().split(" "))
			State.first_num = 0
			if self.waypoint_number == t_n.state_transition_point_0:
				print "state change number 0"
				return State.unknuwn#You can change state as you like.
			elif self.waypoint_number == t_n.state_transition_point_1:
				print "state change number 1"
				return State.unknuwn#You can change state as you like.
			elif self.waypoint_number == t_n.state_transition_point_2:
				print "state change number 2"
				return State.unknuwn#You can change state as you like.
			elif self.waypoint_number == t_n.state_transition_point_3:
				print "state change number 3"
				return State.unknuwn#You can change state as you like.
			if 	Robot.specific_navigation_flg == 1:
				Robot.specific_navigation_flg = 0
				return State.place
			if Robot.guide_to_the_car_flg == 1:
				Robot.guide_to_the_car_flg = 0
				return State.finish
			print "waypoint をcount up します"
			self.waypoint_number += 1
			return State.gowp
		else:
			print "Now Waiting!"
		return State.gowp
		
##!!ここはパラメータを変えるだけで動くようにとりあえずしておいたので実験の時にしっかりと選定して変更する必要がある。!!##
##!!あとは任意のｗｐとの距離を調べて何かする場所がこのタスクで必要かどうかを考えること!!##
	def FollowHuman(self):
		if State.first_num == 0:
			State.first_num = 1#次の状態へ
			#cmd3 = "/usr/bin/picospeaker %s" % "Say stop to exit follow"
			#subprocess.call(cmd3.strip().split(" "))
			time.sleep(0.2)
			follow_cmd = String()
			follow_cmd.data = 'start'
			self.follow_pub.publish(follow_cmd)#e_followに追従指示
			
			print "1番から移行します"
		elif State.first_num == 999:
			speech_long = len(Robot.speech_str)
			print Robot.speech_str
			if speech_long > 25:#15文字以上で処理終了
				return State.Follow
			plausibility_followme = 0#文字列のfollow me っぽさ(もっともらしさ)
			for var in range(0,speech_long):
				if plausibility == 0 and Robot.speech_str[var] == "h":
					plausibility = 1
				elif plausibility == 1 and Robot.speech_str[var] == "a":
					plausibility = 2
				elif plausibility == 1 and Robot.speech_str[var] == "o":
					plausibility = 2
				elif plausibility == 2 and Robot.speech_str[var] == "p":
					plausibilityt = 3
				if plausibility == 3:
					cmd3 = "/usr/bin/picospeaker %s" % "what?"
					subprocess.call(cmd3.strip().split(" "))
					State.first_num = 2#hap または　hopで次へ
		elif State.first_num == 1:
			Robot.say_stop_count += 1
			if Robot.say_stop_count > 30:
				Robot.say_stop_count = 0
				cmd3 = "/usr/bin/picospeaker %s" % "Say stop to exit follow"##変更あり　ちょっと変わる２文分けるべき？　もしくはいらない可能性がある
				subprocess.call(cmd3.strip().split(" "))
				time.sleep(0.3)
				#dist_to_waypoint4 = math.hypot(Robot.x-Robot.waypoint[5][0],Robot.y-Robot.waypoint[5][1])#wp4との距離
				#dist_to_waypoint5 = math.hypot(Robot.x-Robot.waypoint[6][0],Robot.y-Robot.waypoint[6][1])#wp5との距離
				#print dist_to_waypoint5
				#if dist_to_waypoint4 < 0.7 and Robot.wp4_through_flg == 0 and Robot.wp5_through_flg == 0:
				#	Robot.wp4_through_flg = 1
				#elif dist_to_waypoint5 < 0.7 and Robot.wp4_through_flg == 0 and Robot.wp5_through_flg == 0:
				#	Robot.wp5_through_flg = 1
				speech_long = len(Robot.speech_str)
				print Robot.speech_str
				if speech_long > 30:#15文字以上で処理終了
					return State.follow
				plausibility_stop = 0#文字列のstop　っぽさ
				#State.first_num = 3#これはでばっくようです必ず消してくださいまじで
				for var in range(0,speech_long):
					if plausibility_stop == 0 and Robot.speech_str[var] == "s" or plausibility_stop == 0 and Robot.speech_str[var] == "n" :
						plausibility_stop = 1
					elif plausibility_stop == 1 and Robot.speech_str[var] == "t" or plausibility_stop == 1 and Robot.speech_str[var] == "o" :
						plausibility_stop = 2
					#elif plausibility_stop == 2 and Robot.speech_str[var] == "p":
					#	plausibility_stop = 3
				#if plausibility_stop == 3:
				#	State.first_num = 3#stpで次へ
				if plausibility_stop == 2:
					State.first_num = 3
		elif State.first_num == 3:
			self.follow_pub.publish('stop')
			cmd3 = "/usr/bin/picospeaker %s" % "Thank you for guiding me"
			subprocess.call(cmd3.strip().split(" "))
			State.first_num = 0
			return State.branch_off
		return State.follow

	def BranchOff(self):#どこに運ぶかを決定（分岐）します。 ex.Kitchen,Dining or Entrance.
		if State.first_num == 0:
			cmd3 = "/usr/bin/picospeaker %s" % "Please tell me"
			subprocess.call(cmd3.strip().split(" "))
			time.sleep(0.05)
			cmd3 = "/usr/bin/picospeaker %s" % "where to put it"
			subprocess.call(cmd3.strip().split(" "))
			time.sleep(1.0)
			State.first_num = 1
		elif State.first_num == 1:
			print "今入ってきました"
			print Robot.place_str
			#Robot.place_str = "kitchen"#これはデバック用のものであり、これを消すことを忘れないでください！
			speech_long = len(Robot.place_str)
			print Robot.place_str
			if speech_long > 40:#15文字以上で処理終了##変更しなければならない#30
				print "ながいぶんしょうすぎる"
				return State.branch_off
			#place_information = Robot.place_str
			#print place_infomation
			if Robot.place_str == "Kitchen":
				print "出村研究室のキッチン"
				State.first_num = 0
				self.DecideToTheKitchen()
				return State.pick_up
			elif Robot.place_str == "Dining":
				print "出村研究室のダイニング"
				State.first_num = 0
				self.DecideToTheDining()
				return State.pick_up
			elif Robot.place_str == "Entrance":
				print "出村研究室のエントランス"
				State.first_num = 0
				self.DecideToTheEntrance()
				return State.pick_up
			else:
				return State.branch_off
		return State.branch_off
	
	def DecideToTheKitchen(self):
		print "I'll go to the kitchen"
		cmd3 = "/usr/bin/picospeaker %s" % "I'll go to the kitchen"
		subprocess.call(cmd3.strip().split(" "))
		time.sleep(3.0)
		Robot.specific_navigation_flg = 1
		Robot.specific_waypoint[0] = Robot.kitchen_waypoint[0]
		Robot.specific_waypoint[1] = Robot.kitchen_waypoint[1]

	def DecideToTheDining(self):
		print "I'll go to the dining"
		cmd3 = "/usr/bin/picospeaker %s" % "I'll go to the dining"
		subprocess.call(cmd3.strip().split(" "))
		time.sleep(3.0)
		Robot.specific_navigation_flg = 1
		Robot.specific_waypoint[0] = Robot.dining_waypoint[0]
		Robot.specific_waypoint[1] = Robot.dining_waypoint[1]
	
	def DecideToTheEntrance(self):
		cmd3 = "/usr/bin/picospeaker %s" % "I'll go to the entrance"
		subprocess.call(cmd3.strip().split(" "))
		time.sleep(3.0)
		Robot.specific_navigation_flg = 1
		Robot.specific_waypoint[0] = Robot.entrance_waypoint[0]
		Robot.specific_waypoint[1] = Robot.entrance_waypoint[1]
		
	def PickUp(self):#どんな処理になるかわからないので６秒待機して動き出すことにします。
		if State.first_num == 0:
			cmd3 = "/usr/bin/picospeaker %s" % "Please put your bag on my laptop "
			subprocess.call(cmd3.strip().split(" "))
			time.sleep(3.0)
			State.first_num = 1
		elif State.first_num == 1:
			Robot.time_count += 1
			if Robot.time_count > 30:
				State.first_num = 0
				Robot.time_count = 0
				return State.gowp
		return State.pick_up
		
	def Place(self):
		if State.first_num == 0:
			cmd3 = "/usr/bin/picospeaker %s" % "Please take this bag"
			subprocess.call(cmd3.strip().split(" "))
			time.sleep(3.0)
			State.first_num = 1
		elif State.first_num == 1:
			Robot.time_count += 1
			if Robot.time_count > 30:
				State.first_num = 0
				Robot.time_count = 0
				return State.ask
		return State.place
		
	def AskForHelp(self):
		if State.first_num == 0:
			cmd3 = "/usr/bin/picospeaker %s" % "Please help me"
			subprocess.call(cmd3.strip().split(" "))
			time.sleep(3.0)
			State.first_num = 1
		elif State.first_num == 1:
			Robot.time_count += 1
			if Robot.time_count > 30:
				State.first_num = 0
				Robot.time_count = 0
				cmd3 = "/usr/bin/picospeaker %s" % "follow me"
				subprocess.call(cmd3.strip().split(" "))
				time.sleep(3.0)
				#Robot.guide_to_the_car_flg = 1##これは本番用です。このフラグが立っている時に車の場所まで行ってくれます。
				return State.gowp
		return State.ask
		
	def RememberTheCarPlace(self):
		if State.first_num == 0:
			State.first_num = 1
			
		elif State.first_num == 1:
			Robot.car_waypoint[0] = Robot.x
			Robot.car_waypoint[1] = Robot.y
			State.first_num = 0
		return State.remember
		
	def Wait(self):
		if State.first_num == 0:
			State.first_num = 1
			cmd3 = "/usr/bin/picospeaker %s" % "Say follow me"##変更あり　ちょっと変わる２文分けるべき？　もしくはいらない可能性がある
			subprocess.call(cmd3.strip().split(" "))
			time.sleep(0.3)
			
		elif State.first_num == 1:
			Robot.say_stop_count += 1
			if Robot.say_stop_count > 30:
				Robot.say_stop_count = 0
				cmd3 = "/usr/bin/picospeaker %s" % "Say follow me"##変更あり　ちょっと変わる２文分けるべき？　もしくはいらない可能性がある
				subprocess.call(cmd3.strip().split(" "))
				time.sleep(0.3)
				speech_long = len(Robot.speech_str)
				print Robot.speech_str
				if speech_long > 30:#15文字以上で処理終了
					return State.wait
				plausibility_follow = 0#文字列のfollow　っぽさ
				for var in range(0,speech_long):
					if plausibility_follow == 0 and Robot.speech_str[var] == "f" or plausibility_follow == 0 and Robot.speech_str[var] == "n" :
						plausibility_follow = 1
					elif plausibility_follow == 1 and Robot.speech_str[var] == "l" or plausibility_follow == 1 and Robot.speech_str[var] == "o" :
						plausibility_follow = 2
				if plausibility_follow == 2:
					State.first_num = 3
		elif State.first_num == 3:
			cmd3 = "/usr/bin/picospeaker %s" % "I start follow you"
			subprocess.call(cmd3.strip().split(" "))
			State.first_num = 0
			return State.follow
		return State.wait
			
		
if __name__ == '__main__':
	rospy.init_node('help_me_carry_i_navigation')#nodeの初期化
	operate = State.wait
	i_nav = I_NAV()
	cold_start = 1
	while(cold_start):#1秒待機
		#cmd3 = "/usr/bin/picospeaker %s" % "I start follow you"
		#subprocess.call(cmd3.strip().split(" "))
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
		print "operate is %d" % operate
		print "State.first_num is %d" % State.first_num
		print "Robot.specific_navigation_flg is %d" % Robot.specific_navigation_flg
		if i_nav.waypoint_number == len(Robot.waypoint):
			print "waypoint_number is %d" % i_nav.waypoint_number
			print "Navigation finished"
			cmd3 = "/usr/bin/picospeaker %s" % "I finish guiding"
			subprocess.call(cmd3.strip().split(" "))
			operate = State.finish
		if operate == State.finish:
			sys.exit()
		elif operate == State.door:#You mast add new state that you want. 
			print "door"
			operate = i_nav.DoorOpenDetection()
		elif operate == State.gowp:
			print "navigaton"
			operate = i_nav.NavigateWp()
		elif operate == State.follow:
			print"follow"
			operate = i_nav.FollowHuman()
		elif operate == State.branch_off:
			print"Branch off destination"
			operate = i_nav.BranchOff()
		elif operate == State.pick_up:
			print"Pick up the bag"
			operate = i_nav.PickUp()
		elif operate == State.place:
			operate = i_nav.Place()
		elif operate == State.ask:
			operate = i_nav.AskForHelp()
		elif operate == State.remember:
			operate = RememberTheCarPlace()
		elif operate == State.wait:
			operate = i_nav.Wait()#follow me を認識
		
		
		rate = rospy.Rate(5)
		rate.sleep()
