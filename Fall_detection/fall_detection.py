#!/usr/bin/env python3
"""
ROS2 version of fall_detection with single NODE.
Author:Pranav C															   
"""
import rclpy, torch, cv2, os, time, numpy as np, requests, json
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from threading import Thread
from custom_msgs.msg import FleetNotification, IpadMedia
from ament_index_python.packages import get_package_share_directory
from utilities.editor_tools import check_new_image, global_parameters, get_file_name
from os.path import join

class Fall_detection(Node):
	def __init__(self, node_name):
		super().__init__(node_name)
		self.declare_param()
		self.base_url = global_parameters.get("fleet_base_url")
		self.robot_name = global_parameters.get("robot_name") 
		self.fleet_image_link = global_parameters.get("fleet_image_base_link")
		self.initial_values()
		self.fall_logs_csv_path = "/home/rospc/robot_data/collected_data/fall_logs.csv"
		self.height = self.get_parameter("image_height").value
		self.width = self.get_parameter("image_width").value
		self.save_path = join(get_package_share_directory('invento_vision'),'saved_files','images')+"/"
		
		self.Confidence_threshold = self.get_parameter("prediction_confidence_threshold").value
		self.y_threshold = int(self.get_parameter("y_cut_off").value*self.height)
		self.human_class = self.get_parameter("human_class_index").value
		self.aspect_threshold = self.get_parameter("aspect_ratio_threshold").value
		self.fall_time_threshold = self.get_parameter("fall_time_threshold").value
		self.payload_type = self.get_parameter("payload_type").value
		self.state = self.get_parameter("starting_state").value
		self.fall_alert_mail_list = self.get_parameter("fall_alert_mail_list").value
		self.yolo_weights_path = join(get_package_share_directory('invento_vision'),'resource','yolov5',self.get_parameter('yolov5_model_complexity').value+'.pt')
		self.model = torch.hub.load('ultralytics/yolov5', 'custom', device='cpu', path=self.yolo_weights_path)  # local model
		self.create_timer(5, self.status_updater)
		self.subscribers()
		self.publishers()

	def initial_values(self):
		self.user_session_id = "181"
		self.t0 = time.time()
		self.fall_check_start_time = time.time()
		self.need_to_send_email = True
		self.n_falls_present = False
		self.current_state = "Beginning FALL DETECTION NODE"
		self.previous_state = "None"
		self.image_upload_url = self.base_url+"/upload/"
		self.email_url = self.base_url+"/mail_user/"

	def declare_param(self):
			self.declare_parameters(
				namespace='',
				parameters=[
					('name',"minimitra_ros2"),
					('fall_time_threshold',2.4),
					('payload_type','camera_feed'),
					('image_height',640),
					('image_width',480),
					('prediction_confidence_threshold',0.55),
					('y_cut_off',0.450),
					('human_class_index',0),
					('aspect_ratio_threshold',0.66),
					('starting_state','alive'),
					('yolov5_model_complexity','yolov5l'),  
					("fall_alert_mail_list", ["vishnu@mitrarobot.com","gautham@mitrarobot.com"])               
					]
			)
  
	def subscribers(self):
		self.new_image_subscriber = self.create_subscription(Bool, 'new_webrtc_image', self.new_image_callback, 1)
		self.fleet_command_subscriber = self.create_subscription(String, 'nav_command_message', self.fleet_command_callback, 10)
		self.session_id_subscriber = self.create_subscription(String, 'user_session_id', self.session_id_callback, 10)
		self.subscriber_thread = Thread(target=rclpy.spin, args=(self,))
		self.subscriber_thread.start()
	
	def publishers(self):
		self.to_ipad = self.create_publisher(IpadMedia,'to_ipad', 1)
		self.fleet_notif = self.create_publisher(FleetNotification,'fleet_notification', 10)
		self.pose_image_pub = self.create_publisher(Image,'pose_image', 10)

	def new_image_callback(self,_):
		self.new_image = True
		
	def session_id_callback(self, data):
		self.user_session_id = str(data.data)
		self.get_logger().info(f"User session id received in fall detection program : {self.user_session_id}")

	def status_updater(self):
		if self.state == "alive":
			if self.current_state != self.previous_state:
				self.get_logger().info(f"[STATE CHANGED]: [Current state]-->{self.current_state}")
				self.previous_state = self.current_state
		

	def fleet_command_callback(self, data):
		strr = str(data.data)
		if strr in ["6003", "6004", "6050"]:
			self.get_logger().info(f"Received valid command {strr} from fleet") 
		if strr.find("6003") != -1:
			self.state = 'alive'
			fleet_message = ('Beginning FALL DETECTION----')
			self.fleet_notif.publish(FleetNotification(type= FleetNotification.FALLDETECTED, message= fleet_message))
		elif (strr.find("6004") != -1) or (strr.find("6050") != -1):
			self.state = 'sleeping'
			fleet_message = ('Stopping FALL DETECTION----')
			if (strr.find("6050") != -1) : fleet_message = "Stopping all CV processes"
			self.fleet_notif.publish(FleetNotification(type= FleetNotification.FALLDETECTED, message= fleet_message))

	def run_prediction(self,image):
		results = self.model(image)
		try:
			labels, cord_thres = results.xyxyn[0][: , -1].cpu().numpy(), results.xyxyn[0][: ,: -1].cpu().numpy()
		except:
			labels, cord_thres = [], []
		return labels, cord_thres
		
	def upload_file(self, file_name, file_path):        
		payload={
					'name': self.robot_name,
					'type': "camera_feed"
					}

		files=[('file',(file_name,open(file_path,'rb'),'image/jpeg'))]
		headers = {}

		response = requests.request("POST", self.image_upload_url, headers=headers, data=payload, files=files)

	def send_email(self, image_name, developer_logs = False):
		self.image_link = self.fleet_image_link+self.robot_name+'/'+self.payload_type+'/'+ image_name
		self.get_logger().info(self.image_link)

		if not developer_logs:
			send_fall_analytics_to_server_url = self.base_url+"/save_fall/"+self.user_session_id  
			payload = json.dumps({
				"url": self.image_link,
				"location": "1,2",
			})
			headers = {
				'Content-Type': 'application/json'
			}
			response = requests.request("POST", send_fall_analytics_to_server_url, headers=headers, data=payload)
			self.get_logger().info(response.text)

		self.img_data = "<img src=" + self.image_link + ">"
		self.htmlText = "<!DOCTYPE html> <head><h3> Potential fall detected. Please verify. </head></h3> <br> <body>" + self.img_data + "</body> <br> <p> Minimitra. </p>"

		if developer_logs:
			payload = json.dumps({
			"email": ['vishnu@mitrarobot.com', 'narasimha@mitrarobot.com'],
			"subject": "Developer logs-Annotated Fall detection.",
			"body": self.htmlText
			})

		else: 
			payload = json.dumps({
			"email": self.fall_alert_mail_list,
			"subject": "Potential fall detected.",
			"body": self.htmlText
			})

		headers = {
		  'Content-Type': 'application/json'
		}
	
		response = requests.request("POST", self.email_url, headers=headers, data=payload)
		self.get_logger().info(response.text)

	def process_predictions(self,image,labels, thresholds):
		self.clean_image = image.copy()
		n_people_present = 0
		n_falls_present = 0
		for label, threshold in zip(labels, thresholds):
			class_number = int(label)
			confidence = threshold[-1]

			if class_number != self.human_class or confidence<self.Confidence_threshold: continue
			n_people_present += 1
			y0 = int(threshold[1]*self.height)
			# self.get_logger().info(f"y0 is : {y0}")

			if y0>self.y_threshold: 
				n_falls_present += 1

		self.n_falls_present = True if n_falls_present>0 else False
		if n_people_present == 0:
			self.current_state = "[NO ONE is PRESENT in the frame]"
		elif self.n_falls_present:
			self.current_state = "[SERIOUS ALERT]: [POTENTIAL FALL DETECTED]---------------------------------------"
			# self.get_logger().info(f"[STABLE]")
		else: self.current_state = "[STABLE]" 
		# self.get_logger().info(f"need to send email: {self.need_to_send_email}")

		if self.n_falls_present: 
			self.get_logger().info(f"Fall confirmation time: {time.time()-self.fall_check_start_time}")
			if ((time.time()-self.fall_check_start_time > self.fall_time_threshold) and self.need_to_send_email):   
				self.get_logger().warning('<<<<<Fall detected>>>>>') 
				self.ann_img = self.analyze_process_predictions(image, labels, thresholds)
				self.ann_file_name = get_file_name("fall_detection", suffix="annotated")
				self.ann_file_path = join(self.save_path,self.ann_file_name)
				cv2.imwrite(self.ann_file_path, self.ann_img)
				self.upload_file(self.ann_file_name, self.ann_file_path)
				self.send_email(self.ann_file_name, developer_logs= True)
				os.remove(self.ann_file_path)
				self.ann_image_link = self.fleet_image_link+self.robot_name+'/'+self.payload_type+'/'+ self.ann_file_name

				self.file_name = get_file_name("fall_detection", suffix="raw")
				self.file_path = self.save_path+self.file_name
				cv2.imwrite(self.file_path, self.clean_image)
				self.upload_file(self.file_name, self.file_path)
				self.send_email(self.file_name, developer_logs= True)
				os.remove(self.file_path)
				self.image_link = self.fleet_image_link+self.robot_name+'/'+self.payload_type+'/'+ self.file_name
				

				# data = [self.score_text, self.min_diagonal, self.ann_image_link ,self.not_enough_inside]
				with open(self.fall_logs_csv_path,'a') as fd:
					fd.write("scores: "+str(self.score_text)+','
					+" fall_prediction_rejected: "+str(self.not_enough_inside)+','
					+" is_corner?" + str(self.corner_of_frame)+ f"(a0,a1)->({self.fall_a0},{self.fall_a1}) "+','
					+" aspect_ratio" + str(self.aspect_ratio)+','
					+" min_diagonal: "+str(self.min_diagonal)+','
					+" image_link: "+self.image_link+','
					+" ann_image_link: "+str(self.ann_image_link)+'\n\n')
				self.need_to_send_email = False

				if not self.not_enough_inside: # otherwise Don't send mails 
					# self.file_name = get_file_name("fall_detection", suffix="raw")
					# self.file_path = join(self.save_path, self.file_name)
					# fleet_message = f"WARNING: FALL DETECTED, time: {self.file_name[:-4]} "
					fleet_message = f"WARNING: FALL DETECTED, Sending E Mail notificaiton"
					self.fleet_notif.publish(FleetNotification(type= FleetNotification.FALLDETECTED, message= fleet_message))
					self.to_ipad.publish(IpadMedia(media = "human fall detected"))
					# time.sleep(1) #TODO: uncomment after fixing that issue
					cv2.imwrite(self.file_path, self.clean_image)
					# self.upload_file(self.file_name, self.file_path)
					self.send_email(self.file_name, developer_logs= False)
					os.remove(self.file_path)

					self.fall_check_start_time = time.time()
					self.need_to_send_email = False
		  
		elif len(labels)>0:
			self.fall_check_start_time = time.time()
			self.need_to_send_email = True
			
		return image 
		  
	@property
	def webrtc_image(self):
		return SharedInfo.get_webrtc_image()		

	def analyze_process_predictions(self,image,labels, thresholds):
		fall_text = score_text = aspect_ratio_text = angles_text = ''
		self.clean_image = image.copy()
		ann_image = image.copy()
		self.corner_of_frame = False
		self.not_enough_inside = False
		self.min_diagonal = np.inf

		image = cv2.line(image.copy(), (0,self.y_threshold), (self.width,self.y_threshold), (255,255,255), 1)
		n_falls_present = 0
		for label, threshold in zip(labels, thresholds):
			
			class_number = int(label)
			confidence = threshold[-1]

			if class_number != self.human_class or confidence<self.Confidence_threshold: continue

			a0,b0,a1,b1 = threshold[:4]
			self.a0, self.a1 = a0, a1

			bboxes = a0*self.width, b0*self.height, a1*self.width, b1*self.height
			bboxes = [int(x) for x in bboxes]

			x0,y0,x1,y1 = bboxes
			if (a0 <= 0.02 or a1 >= 0.98) and (y0>self.y_threshold): self.corner_of_frame = True
			diagonal = np.sqrt((y1-y0)**2+(x1-x0)**2)
			self.min_diagonal = min(self.min_diagonal, diagonal) 

			diagonal_length_threshold = 0.35 if self.corner_of_frame else 0.12          
			self.aspect_ratio = round((y1-y0)/(x1-x0),2)
			aspect_ratio_text += ', ' + str(self.aspect_ratio) #+" "+str(int(100*(1-b1)))+":"+str(int(100*(1-b0)))
			score_text += ', ' + str(np.floor(confidence*100)/100)

			image = cv2.rectangle(image,(x0, y0),(x1, y1),(0,255,0),3)
			image = cv2.circle(image,(x0, y0),10,(255,255,255),-1)
			image = cv2.circle(image,(x1, y1),10,(255,255,255),-1)

			if y0>self.y_threshold: #or 
				n_falls_present += 1 
				self.get_logger().info('confirming fall....')
				det_fall = "Fall "
				self.fall_a0, self.fall_a1 = a0, a1
			else :
				det_fall= "Stable "
				self.get_logger().info('Stable.')

			fall_text += det_fall

			if (y0>self.y_threshold) and ((self.min_diagonal < diagonal_length_threshold*self.width) or (self.corner_of_frame and self.aspect_ratio > 2.6)) : self.not_enough_inside = True
			if (y0>self.y_threshold) and not ((self.min_diagonal < diagonal_length_threshold*self.width) or (self.corner_of_frame and self.aspect_ratio > 2.6)):
				self.not_enough_inside = False
				break
			
		self.get_logger().info(f"need to send email: {self.need_to_send_email}")
		self.n_falls_present = True if n_falls_present>0 else False
		image = cv2.putText(image, fall_text, (0, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
		image = cv2.putText(image, "Conf: "+score_text, (0, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
		# image = cv2.putText(image, "Ang: "+angles_text, (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
		self.score_text = score_text
		return image
						   
	def run(self, analysis_mode = False):
		self.new_image = False
		while True:
			if self.state != "alive" :
				time.sleep(3)
				continue
			if self.webrtc_image is None:
				self.get_logger().warning('[No WEBRTC DATA]')
				time.sleep(1)
				continue
			time.sleep(3)
			check_new_image(self)
			image = self.webrtc_image.copy()
			rgb_img = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
			labels, cord_thres = self.run_prediction(rgb_img)
			if analysis_mode: self.analyze_process_predictions(image, labels,cord_thres)
			else: self.process_predictions(image, labels,cord_thres)
							
def main(args=None):
	global fall
	rclpy.init(args=args)
	fall = Fall_detection("fall_detection")
	fall.run(analysis_mode=False)

if __name__ == "__main__":
	main()
