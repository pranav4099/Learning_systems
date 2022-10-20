#!/usr/bin/env python3
"""
Smile detection node
Author:Pranav C																	   
"""
import cv2, numpy as np, time, glob, json, requests, rclpy
from fer import FER
from std_msgs.msg import String
from deepface import DeepFace
from ros_img_conversion import Invento_img_process
from editor_tools import get_file_name, global_parameters
from rclpy.node import Node
from threading import Thread
from os.path import join

class SmileDetection(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.name = node_name
		self.valid_fleet_commands = ['6009', '6010', '6050']
		self.start_command = "6009"
		self.stop_command = "6010"
		self.declare_param()
		self.get_logger().info(f"Started the node: {node_name}")
		self.FER_detector = FER(mtcnn=True)
		self.prev_emotion = 'neutral'
		self.state = "sleeping"
		self.user_session_id = "182" #************************************* Not yet given!!!!!!!!!!
		self.counter = 0
		self.happy_confirm_counter = 0
		self.start_time = time.time()
		self.image_save_path =  join(get_package_share_directory('vision'), 'saved_files','images','smiles')+'/'
		self.collage_save_path =  join(get_package_share_directory('vision'), 'saved_files','images','collages')+'/'
		# self.width, self.height = 160, 120
		self.mini_width, self.mini_height = self.get_parameter("image_width").value/4, self.get_parameter("image_height").value/4
		self.robot_name = global_parameters.get("robot_name")
		self.base_url = global_parameters.get("fleet_base_url")
		self.fleet_image_link = global_parameters.get("fleet_image_base_link")
		self.image_upload_url = self.base_url+"/upload/"
		self.payload_type = self.get_parameter("payload_type").value
		self.number_of_images = 0
		self.idle_latch = None
		# self.webrtc_image = None
		self.create_timer(5, self.publish_robot_state)
		self.invento_custom_cv_bridge = Invento_img_process()
		self.subscribers()        
		self.publishers()

	def declare_param(self):
			self.declare_parameters(
				namespace='',
				parameters=[
					('payload_type','camera_feed'),
					('image_height',640),
					('image_width',480),
					]
			)
	

	def publishers(self):
		self.robot_state_publisher = self.create_publisher(String, 'robot_state', 10)
		self.fleet_notif = self.create_publisher(String, 'txt_msg', 10)
		self.smile_detection_publisher = self.create_publisher(String, "txt_msg", 10)
		self.smile_count_publisher = self.create_publisher(String, "smile_count", 10)

	def subscribers(self):
		self.session_id_subscriber = self.create_subscription(String, 'user_session_id', self.session_id_callback, 10)
		self.fleet_command_subscriber = self.create_subscription(String, 'nav_command_message', self.fleet_command_callback, 10)
		self.subscriber_thread = Thread(target=rclpy.spin, args=(self,))
		self.subscriber_thread.start()

	def fleet_command_callback(self, data):
		strr = str(data.data)
		if strr in self.valid_fleet_commands:
			self.get_logger().info(f"Received valid command {strr} from fleet") 
		if strr.find(self.start_command) != -1:
			self.state = 'alive'
			fleet_message = (f'Beginning {self.name}----')
			self.fleet_notif.publish(String(data=fleet_message))
		elif strr.find(self.stop_command) != -1 or (strr.find("6050") != -1):
			self.state = 'sleeping'
			self.counter = 0
			fleet_message = (f'Stopping {self.name}----')
			if (strr.find("6050") != -1) : fleet_message = "Stopping all CV processes"
			self.fleet_notif.publish(String(data=fleet_message))

	def publish_robot_state(self):
		robot_state = "Vision" if self.state == "alive" else "Idle"
		if robot_state=="Idle":
			if not self.idle_latch: 
				self.robot_state_publisher.publish(String(data="Idle"))
				self.idle_latch = True
			return
		else: 
			self.idle_latch = False
			self.robot_state_publisher.publish((String(data=robot_state)))

	def session_id_callback(self, data):
		self.user_session_id = str(data.data)
		self.get_logger().info(f"User session id received in fall detection program : {self.user_session_id}")
	
	def upload_file(self, image, file_name):     #?--------------------------change this based on THE API That ASWIN bro gives-------------------------------------------------   
		image_bytes = cv2.imencode('.jpg', image)[1].tobytes()
		payload={
					'name': self.robot_name,
					'type': "camera_feed"
					}

		files=[('file',(file_name,image_bytes,'image/jpeg'))]
		headers = {}

		response = requests.request("POST", self.image_upload_url, headers=headers, data=payload, files=files)
		self.get_logger().info(f"Image upload response.text: {response.text}")

		self.image_link = self.fleet_image_link+self.robot_name+'/'+self.payload_type+'/'+ file_name
		self.get_logger().info(f"Image_link: {self.image_link}")
		send_smile_analytics_to_server_url = self.base_url+"/save_smile/"+self.user_session_id  

		self.get_logger().info(f'Sending smile analytics to server in url : {send_smile_analytics_to_server_url}')
		payload = json.dumps({
			"url": self.image_link,
			"location": "1,2",
		})
		headers = {
			'Content-Type': 'application/json'
		}
		response = requests.request("POST", send_smile_analytics_to_server_url, headers=headers, data=payload)
		self.get_logger().info(response.text)
		self.get_logger().info("Smile Detection JSON sent to server!")

	@property
	def webrtc_image(self):
		return SharedInfo.get_webrtc_image()

	def create_collage(self):
		blank_image = np.zeros((self.height*3,self.width*5,3), np.uint8) #* Changed it to a grid of 3 rows and 5 columns
		c = 0
		h_start = w_start = 0 
		for filename in glob.iglob(self.image_save_path+'*.jpg',recursive = False):
			self.get_logger().info(filename)
			image = cv2.imread(filename)
			image = cv2.resize(image,(self.width,self.height))
			h_end = h_start+self.height
			w_end = w_start+self.width
			c+=1
			if c <= 15:   #* collage of 15 IMAGES
				blank_image[h_start:h_end, w_start:w_end] = image
			else:
				break
			w_start = w_end
			# if w_start == 640:
			if w_start == self.width*5:
				w_start = 0
				h_start += self.height
		self.get_logger().info("Collage created")
		self.get_logger().info("\n\n\n\n")
		collage_filename = get_file_name("smile_detection", suffix="collage", ext="jpeg")
		# collage_filename = 'collage'+str(time.time())+'.jpeg'
		self.get_logger().info(f"collage name is : {collage_filename}")
		self.upload_file(blank_image, collage_filename) #* upload collage to server but this needs to be handled differently!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


	def run_detections(self):
		self.get_logger().info('------------------------Runnning Detections-----------------------------')
		self.prev_state = None
		while True:
			time.sleep(0.15)
			if self.webrtc_image is None:
				self.get_logger().info(f"No webrtc image")
				time.sleep(1)
				continue
						 
			if self.state == 'sleeping':
				if self.prev_state != 'sleeping':
					self.get_logger().info(f"{self.name} is sleeping")
					self.prev_state = 'sleeping'
				time.sleep(1)
				continue
		
			self.prev_state = 'alive'
			frame = self.webrtc_image
			og_frame = frame.copy()
			model_type = 'FER'
			if model_type == "deepface":
				try:
					obj = DeepFace.analyze(img_path = frame, actions = ['emotion']) #* original version
					# obj = DeepFace.analyze(img_path = frame, actions = ['emotion'], models = models) #* supposed to be faster version
					#* change other parameters, try out different models to confirm the best one!   
					dominant_emotion = obj['dominant_emotion']
					emotion = 'happy' if dominant_emotion == 'happy' else 'neutral'
					happy_score = int(obj['emotion']['happy'])
					neutral_score = int(obj['emotion']['neutral'])
					dominant_score = int(obj['emotion'][dominant_emotion])
					# if (happy_score>20) and (happy_score > neutral_score) : emotion = 'happy' #* Check if this is wrong and can glitch out!!!!!! - IT does glitch out! 
					# self.get_logger().debug(int(obj['emotion']['happy']),emotion)
					self.get_logger().info(f"Dominant emotion: {dominant_emotion} with confidence : {dominant_score}")
					self.get_logger().info(f"Happy confidence: {happy_score}, Neutral confidence: {neutral_score}, Emotion is : {emotion}")
				except Exception as e:
					# self.get_logger().debug(f"No face reaction detected")
					self.get_logger().info(f"No face reaction detected")
					time.sleep(1)
					emotion = 'neutral'
				# self.get_logger().debug(emotion)
				
			elif model_type == "FER":
				res = self.FER_detector.detect_emotions(frame)
				if res == []: 
					self.no_face = True
					self.get_logger().info(f"No face reaction detected")
					emotion = "neutral"
				else:
					self.no_face = False    					
					print(f"res: {res}")
					happy = int(res[0]['emotions']['happy']*100)
					neutral = int(res[0]['emotions']['neutral']*100)
					print(f"happy: {happy}, neutral: {neutral}")
					emotion = 'happy' if happy>neutral else 'neutral'

			self.get_logger().info(emotion)

			if emotion == 'neutral' and self.prev_emotion == 'happy':
				self.get_logger().info("Transition from happy to netural, setting happy_confirm_counter to 0")
				self.happy_confirm_counter = 0
			elif emotion == 'happy' and self.prev_emotion == 'happy':
				self.happy_confirm_counter += 1
				self.get_logger().debug(f"happy_confirm_counter is {self.happy_confirm_counter}")

				if self.happy_confirm_counter>=2:
					self.counter+=1
					self.happy_confirm_counter = 0 #* reset counter
					self.get_logger().debug(f"smile {self.counter} detected, Now sleeping for 3 seconds......................") 
					self.smile_detection_publisher.publish(String(data="smile_detected"))
					self.get_logger().info(f"Publishing smile count: {str(self.counter)}")
					self.smile_count_publisher.publish(String(data=str(self.counter)))
					time.sleep(1)
					self.file_name = 'smile'+str(self.counter)+'.jpg'
					self.upload_file(og_frame, self.file_name)
					time.sleep(3)
					self.get_logger().debug("I'm awake now!!!")
				   
				if self.counter>=15:
					try:
						self.create_collage()
					except:
						self.get_logger().error("Create collage error!!!")
						pass
					self.counter = 0

			self.prev_emotion = emotion
			cv2.putText(frame,str(self.counter),(10,100),cv2.FONT_HERSHEY_DUPLEX,4,(0,255,255),3)

def main(args=None):
	rclpy.init(args=args)
	smile_node = SmileDetection("SmileDetection")
	smile_node.run_detections()

if __name__=='__main__':
	main()
