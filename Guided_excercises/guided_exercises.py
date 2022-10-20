#!/usr/bin/env python3
"""
ROS2 guided exercises with a single NODE.
Author:Pranav c
"""
import cv2, time, os, json, requests, mediapipe as mp, numpy as np
from utilities.editor_tools import send_to_ipad
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool
import rclpy
from os.path import join
from rclpy.node import Node
from threading import Thread


class Guided_exercises(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.name = node_name
		self.declare_param()
		self.fixed_constants()
		self.initial_values()
		self.ros2_subscribers()
		self.create_timer(10, self.publish_robot_state)
		self.ros2_publishers()
	
	def fixed_constants(self):
		self.get_angle_data = Proc_pose()
		self.mp_drawing = mp.solutions.drawing_utils
		self.invento_custom_cv_bridge = Invento_img_process()
		self.mp_pose = mp.solutions.pose
		self.joints = ["lefthip","righthip","leftknee","rightknee","leftshoulder","rightshoulder","leftelbow","rightelbow"]
		self.list_of_error_tolerances = {'righthip':0.8,'lefthip':0.8,'rightknee':0.8,'leftknee':0.8,'rightshoulder':1.0,'leftshoulder':1.0,'rightelbow':1.2,'leftelbow':1.2}
		
	def initial_values(self):
		self.idle_latch = None
		self.timer = time.time()-1 #these two lines are to ensure that immediately we get the first message! 
		self.timer_start = False
		self.success_msg = ''
		self.reference_exercise_image = None
		self.selected_exercise = "None2"
		self.current_exercise = "None"
		self.x_name = self.current_exercise.split('.')[0]
		self.base_imager_folder_url = os.path.join(get_package_share_directory('invento_vision'), 'resource','reference_images')+'/'

	def declare_param(self):
		self.declare_parameters(
			namespace='',
			parameters=[
				('starting_state', 'sleeping'),
				('image_width',940),
				('image_height',900),
				('interval_between_msgs', 2.5),
				('default_exercise', "arms.jpeg")
				]
		)
		self.state_trigger = self.get_parameter("starting_state").value
		self.width = self.get_parameter('image_width').value 
		self.height = self.get_parameter('image_height').value 
		self.interval_between_msgs = self.get_parameter('interval_between_msgs').value
		 

	def ros2_subscribers(self):
		self.fleet_cmd_subscriber = self.create_subscription(String, 'nav_command_message', self.received_fleet_cmd, 10)
		self.ipad_cmd_subscriber = self.create_subscription(String, 'from_ipad', self.received_ipad_cmd, 10)
		self.get_logger().info(f'--------[ SUBSCRIBERS INITIALIZED ]---------')
	
	def ros2_publishers(self):
		self.fleet_notif = self.create_publisher(String, 'txt_msg', 10)
		self.robot_state_publisher = self.create_publisher(String, 'robot_state', 10)
		self.new_pose_image_pub = self.create_publisher(Bool,'new_pose_image', 10) #TODO: Change this from sending images to sending only the Boolean in ROS and pose image in SharedInfo
		self.ipad_publisher = self.create_publisher(IpadMedia,'to_ipad', 10)
		self.weighing_done_pub = self.create_publisher(String, "weighing_done", 10)
		self.nav_command_publisher = self.create_publisher(String, 'nav_command_message', 10)

	def publish_robot_state(self):
		robot_state = "Vision" if self.state_trigger == "alive" else "Idle"
		if robot_state=="Idle":
			if not self.idle_latch: 
				self.robot_state_publisher.publish(String(data="Idle"))
				self.idle_latch = True
			return
		else: 
			self.idle_latch = False
			self.robot_state_publisher.publish((String(data=robot_state)))

	
	@property
	def webrtc_image(self):
		return SharedInfo.get_webrtc_image()

	def received_fleet_cmd(self, data):
		self.command_callback(data, from_ipad=False)

	def received_ipad_cmd(self, data):
		self.command_callback(data, from_ipad=True)
		
	def command_callback(self, data, from_ipad):
		try: 
			numeric_command = int(data.data) # if a number
			if numeric_command not in [6005,6006,6050]: return # invalid command
			self.get_logger().info(f"Received [VALID] [NUMERIC-COMMAND] [FROM FLEET] : {numeric_command}")
			if numeric_command in [6006, 6050]: # Stop command
				self.state_trigger = "sleeping"
				fleet_message = f'Ending Guided exercise: {self.current_exercise} ----'
				if numeric_command == 6050: fleet_message = 'Stopping all CV processes'
				self.fleet_notif.publish(String(data=fleet_message))
				return

			self.nav_command_publisher.publish(String(data="6000"))
			self.state_trigger = "alive"

			self.success_msg = 'Have a good time!'
			fleet_message = f'Beginning Guided exercise: {self.current_exercise} ----'
			self.fleet_notif.publish(String(data=fleet_message))
			return
		except:
			if data.data == "reset":
				self.state_trigger = "sleeping"
				self.get_logger().info(f"Received [RESET] [STRING-COMMAND] [FROM IPAD] : {data.data}")
				self.fleet_notif.publish(String(data=f'Ending Guided exercise: {self.current_exercise} ----'))
				return
				 
			strr = data.data
			key_word = strr.split('.')[0]
			if key_word != "exercise": return 
			try: x_name = strr.split('.')[1] 
			except: 
				self.get_logger().warning(f"[INVALID EXERCISE COMMAND] : {strr}")
				return

			self.nav_command_publisher.publish(String(data="6000"))
			self.state_trigger = "alive"
			self.success_msg = 'Exercise changed!'

			if from_ipad: 
				self.weighing_done_pub.publish(data) 
				self.get_logger().info(f"Received [EXERCISE COMMAND] [FROM IPAD] : {strr}")

			else: self.get_logger().info(f"Received [EXERCISE COMMAND] [FROM FLEET] : {strr}")
				
			self.selected_exercise_list = [x_name + '.jpeg', x_name + '.jpg', x_name + '.png']
			for self.selected_exercise in self.selected_exercise_list:
				if os.path.exists(self.base_imager_folder_url + self.selected_exercise): break
			return			
	
	def select_exercise(self):
		if self.selected_exercise != self.current_exercise:

			self.image_url = self.base_imager_folder_url + self.selected_exercise
			self.nav_command_publisher.publish(String(data="6000")) 
			if os.path.exists(self.base_imager_folder_url + self.selected_exercise): 
				self.reference_exercise_image = cv2.resize(cv2.imread(self.image_url), (self.width, self.height))
			else:
				if self.reference_exercise_image is None:
					self.image_url = self.base_imager_folder_url + "arms.jpeg"
					self.reference_exercise_image = cv2.resize(cv2.imread(self.image_url), (self.width, self.height))
					self.selected_exercise = "arms.jpeg"
					self.get_logger().info(f'Chosen exercise not found, using default exercise: [{self.selected_exercise}]')
				else:
					self.selected_exercise = self.current_exercise
					self.get_logger().info(f'Chosen exercise not found, using the previous exercise: [{self.selected_exercise}] itself!') 
				return

			thresholds_json_object = open("/home/rospc/ros2_ws/navigation-ros2/src/packages/invento_vision/resource/guided_exercise_thresholds.json")
			self.exercises = json.load(thresholds_json_object)
			thresholds_json_object.close() 

			self.current_exercise = self.selected_exercise 
			self.x_name = self.current_exercise.split('.')[0]
			self.exercise_image_pose = HeavyStaticPose.get_static_pose(self.reference_exercise_image,return_image=False)    
			_,self.ref_image_angle_data = self.get_angle_data.pose_angles(self.exercise_image_pose.pose_landmarks, self.reference_exercise_image)
			self.current_list_of_error_tolerances = self.list_of_error_tolerances.copy()
			self.current_key_joints = self.joints.copy()
			if self.x_name in self.exercises:
				key_joints = self.exercises[self.x_name]['key_joints']
				self.get_logger().info(f'xname, key_joints in 214: {self.x_name}, {key_joints}')
				alter_these_thresholds = self.exercises[self.x_name]['alter_these_thresholds']
				
				self.current_key_joints = key_joints 
				for joint, altered_value in alter_these_thresholds.items():
					self.current_list_of_error_tolerances[joint] = altered_value
			self.get_logger().info(f"self.current_key_joints : {self.current_key_joints}")
			self.get_logger().info(f"current threshold values: {self.current_list_of_error_tolerances}")

			self.mp_drawing.draw_landmarks(
				self.reference_exercise_image, self.exercise_image_pose.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

			self.get_logger().info(f'exercise successfully changed to: {self.selected_exercise}' )
			self.state_trigger = "alive"

	def run(self):
		with self.mp_pose.Pose(min_detection_confidence=0.65,
							   model_complexity= 1,
							   min_tracking_confidence=0.5) as pose:

			while True:
				time.sleep(0.125) #?------- see what sleep is ideal! This sleep is for webrtc feed to look good, but in the end, we should only bother about the IPAD mode, or we can tune the sleep based on who triggers it!   
				if self.state_trigger == "alive":
					self.select_exercise()
					difference=[]
					if self.webrtc_image is None:
						self.get_logger().info('no webrtc input.')
						time.sleep(1)
						continue  
					
					self.select_exercise()

					image = cv2.cvtColor(self.webrtc_image, cv2.COLOR_BGR2RGB) #?-------------I think we don't have to flip it before sending! naf
					image.flags.writeable = False
					results = pose.process(image)
					image.flags.writeable = True
					image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
					pose_landmarks = results.pose_landmarks


					if pose_landmarks is not None:
						lst_coordinates,camera_image_angle_data = self.get_angle_data.pose_angles(pose_landmarks, image)
						zip_object = zip(self.ref_image_angle_data,camera_image_angle_data)
						
						idx_of_errors = []
						cntr = 0

						for list1_i, list2_i in zip_object:
							error = list1_i-list2_i
							difference.append(np.abs(error))
							idx_of_errors.append(cntr)
							cntr += 1

						if self.timer_start == True:
							self.timer = time.time()
							self.timer_start = False

						if self.timer_start == False and int(time.time() - self.timer) > self.interval_between_msgs:
							success = True
							self.success_msg = 'FAILED due to: '
							for idx in idx_of_errors:
								current_joint = self.joints[idx]
								self.get_logger().info(f"error in {current_joint} : {difference[idx]}")
								if not current_joint in  self.current_key_joints: continue
								if difference[idx] > self.current_list_of_error_tolerances[current_joint]:
									self.success_msg += f"{current_joint}  "
									self.get_logger().info(f"correct your {current_joint}! error: {difference[idx]}")
									msg = f"exercise.{self.x_name}.bodypose.{current_joint}"
									send_to_ipad(self, self.ipad_publisher, msg, 'Exercise')
									self.timer_start = True
									success = False
									break

							if success:
								self.success_msg = 'Accomplished.'
								msg = "exercise.bodypose.done"
								send_to_ipad(self, self.ipad_publisher, msg, 'Exercise')
								self.timer = time.time()-1 #these two lines are to ensure that immediately we get the first message! 
								self.timer_start = False
								self.state_trigger = "sleeping"
								self.current_exercise = "None"     
						
						if idx_of_errors is not None:
							for idx in idx_of_errors:   
								if idx < 2: #* Correspond to hip angles
									cv2.circle(image,tuple(lst_coordinates[idx]),min(15,int(np.abs(difference[idx])*15/1.5)),(255,255,255),-1) #?------------- 12 is put as the cut off to avoid infinite radius circles! 
								cv2.circle(image,tuple(lst_coordinates[idx]),min(15,int(np.abs(difference[idx])*15)),(255,255,255),-1) #?------------- 12 is put as the cut off to avoid infinite radius circles! 

						self.mp_drawing.draw_landmarks(
							image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
					
					elif pose_landmarks is None:
						msg = "exercise.bodypose.no_one"
						if self.timer_start == False and int(time.time() - self.timer) > self.interval_between_msgs:
							send_to_ipad(self, self.ipad_publisher, msg, 'Exercise')
							self.timer_start = True
						
					image = cv2.resize(image, (self.width, self.height))
					if pose_landmarks is not None:
						image = cv2.putText(image, self.success_msg, (int(self.width/8), int(self.height/7)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 3)
					concat_img = cv2.hconcat([image, self.reference_exercise_image])
					SharedInfo.put_pose_image(concat_img)
					self.new_pose_image_pub.publish(Bool(data=True))

				else:
					time.sleep(1)
					continue



class Subscriber_thread(Thread):
	def __init__(self):
		Thread.__init__(self)

	def run(self, args=None):
		global exercise
		exercise.get_logger().info('____________________________________________________________________')
		rclpy.spin(exercise)
		exercise.destroy_node()

def main(args=None):
	rclpy.init(args=args)
	global exercise	
	exercise = Guided_exercises("guided_exercises")
	subscriber_thread = Subscriber_thread()
	subscriber_thread.start()
	exercise.run()

if __name__ == '__main__':
   main()