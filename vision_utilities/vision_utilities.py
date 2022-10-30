import torch, cv2, os, time, numpy as np
from std_msgs.msg import String
from enum import Enum
from threading import Thread
from editor_tools import get_file_name
from ament_index_python.packages import get_package_share_directory

from fastai.vision.all import parent_label
def get_y(x):
	return [parent_label(x)]

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

def is_person_using_bed(self):
	percentage_inside_bed = 0              #? default value is 0
	human_a0, human_b0, human_a1, human_b1 = self.person_of_interest
	if self.n_people_present >= 1:	       #? if there is a human in the image
		for bed in self.beds:    	       #? check if the human is inside any of the beds
			human_length = human_a1-human_a0 
			human_height = human_b1-human_b0
			if (bed[0]-human_a0)>= (0.15*human_length) or (human_a1-bed[2])>= (0.15*human_length):  #? if either of the sides, in x-co-ordinates of the human, is outside the bed
				continue                                                                                      #? then it is not inside the bed for sure
			else:
				if human_b1 - bed[3] <= 0.12*(human_height):                                             #? if x- co ordinates, match, check for the y-co-ordinates, since there are beds on which humans can sleep and get detected as fully above the bed, we only check if the bottom box of human is above the bed's bottom most box
					percentage_inside_bed = 1- max(0,(bed[0]-human_a0)/human_length) - max(0,(human_a1-bed[2])/human_length) - max(0,(human_b1-bed[3])/human_height)  #? this isn't the actual percentage inside the bed, but an indicator variable, that penalizes for either x or y being sligthly outside the bed
					break
	using_bed = True if percentage_inside_bed >= 0.6 else False
	return using_bed, percentage_inside_bed

#TODO: have a centralized enum in the vision controller that precisely potrays the visual state of the the universe visible to the robot! With context and passt information taken into consideration. 
class State(Enum):
	"""
	State of the node
	"""
	NO_ONE_FOUND = 0
	MOVING = 1
	NOT_MOVING = 2
	BEGINNING_THE_PROGRAM = 3
	SOMEONE_FOUND = 4
	PROGRAM_NOT_RUNNING = 9

class ProcessPose: #TODO: Not used yet!
	"""Class for processing Mediapipe pose_landmarks."""
	joints = {"heel": 29, "toe": 31, "knee": 25, "ankle": 27, "hip": 23, "shoulder": 11, "elbow": 13, "wrist": 15}
	@staticmethod
	def find_angles(p1,p2,p3):
		Px, Py = np.subtract(p2,p1)
		Qx, Qy = np.subtract(p3,p2)

		theta = abs(np.rad2deg(np.arctan2(Py,Px) - np.arctan2(Qy,Qx)))
		theta = min(theta, abs(360 - theta))
	
		return theta

	@classmethod
	def pose_angles(cls, pose_landmarks, image_size:tuple):
		"""Returns joint locations and joint angles for the given pose_landmarks.
		\nArgs: ( mediapipe Pose landmarks, image_size: tuple of (width,height) )"""
		width, height = image_size
		joint_locations = {}
		for joint, index in cls.joints.items():
			joint_locations[f"left_{joint}"] = np.array(int(width*pose_landmarks[index].x), int(height*pose_landmarks[index].y))
			joint_locations[f"right_{joint}"] = np.array(int(width*pose_landmarks[index+1].x), int(height*pose_landmarks[index+1].y)) #? index of Right Joint is "+1" from Left Joint

		joint_locations["nose"] = np.array(int(width*pose_landmarks[0].x), int(height*pose_landmarks[0].y))

		key_joints = {"knee": ("ankle", "hip"), "hip": ("knee", "shoulder"), "shoulder": ("hip", "elbow"), "elbow": ("shoulder", "wrist")} #? Joint: (joint_below, joint_above)
		joint_angles = {}
		for joint,(joint_below, joint_above) in key_joints.items():			
			joint_angles[f"left_{joint}"] = cls.find_angles(joint_locations[f"left_{joint_below}"], joint_locations[f"left_{joint}"], joint_locations[f"left_{joint_above}"])
			joint_angles[f"right_{joint}"] = cls.find_angles(joint_locations[f"right_{joint_below}"], joint_locations[f"right_{joint}"], joint_locations[f"right_{joint_above}"])	

		return joint_locations, joint_angles

def session_id_callback(self, data):
		self.user_session_id = str(data.data)
		print(f"User session id received in {self.name}: {self.user_session_id}")

def fleet_command_callback(self, data):
	strr = str(data.data)
	self.get_logger().info(f"Fleet command received in {self.name} : {strr}")
	if strr.find(self.fleet_command_start) != -1:
		if self.state_trigger == "start":
			self.get_logger().info(f"{self.name} is already running!")
			self.fleet_notif.publish(String(data=f"{self.name} is already running!"))
			time.sleep(1)
			return
		self.state_trigger = 'start'
		self.prev_state ="dummy"
		fleet_message = (f'Beginning {self.name}----')
		self.fleet_notif.publish(String(data=fleet_message))
		time.sleep(1)
		run_thread = Thread(target = self.run, args =())
		run_thread.start()

	elif (strr.find(self.fleet_command_stop) != -1) or (strr.find("6050") != -1):
		if self.state_trigger != "start":
			print(f"{self.name} is not running!")
			self.fleet_notif.publish(String(data=f"{self.name} is not running!"))
			time.sleep(1)
			return
		self.state_trigger = 'stop'
		fleet_message = (f'Stopping {self.name}----')
		if (strr.find("6050") != -1) : fleet_message = "Stopping all CV processes"
		self.fleet_notif.publish(String(data=fleet_message))
		time.sleep(1)

def run_yolo_prediction(self, image):
		results = self.model(image)
		try:
			labels, cord_thres = results.xyxyn[0][: , -1].cpu().numpy(), results.xyxyn[0][: ,: -1].cpu().numpy()
		except:
			labels =  cord_thres = []
		return labels, cord_thres

def draw_yolo_bounding_box(self, threshold):
		#? Drawing bounding boxes on this image - CAN BE MOVED OUT
		a0,b0,a1,b1 = threshold[:4]
		bboxes = a0*self.width, b0*self.height, a1*self.width, b1*self.height
		bboxes = [int(x) for x in bboxes]
		x0,y0,x1,y1 = bboxes
		self.annotated_image = cv2.rectangle(self.annotated_image,(x0, y0),(x1, y1),(0,255,0),3)
		self.annotated_image = cv2.circle(self.annotated_image,(x0, y0),10,(255,255,255),-1)
		self.annotated_image = cv2.circle(self.annotated_image,(x1, y1),10,(255,255,255),-1)
		#?-------------------------------------------------------------------------------------------------

def is_ready(self):
		if self.state_trigger == 'stop': return False
		if self.webrtc_image is None:
			print(f"No webrtc image received in {self.name}!")
			time.sleep(1)           #? Sleep for a second before trying again for webrtc image 
			return False
		return True                 #? When there are no issues return Ready, but then, check if we need to enforce additional checks to confirm readiness!

#?? DO SOMETHING ABOUT FPS
class Video_recorder_thread(Thread):
	def __init__(self, size, node, save_path= os.path.join(get_package_share_directory('invento_vision'), 'saved_files','videos')+'/', fps = 10): #? FPS?????----------------------------
		Thread.__init__(self)
		self.node = node
		self.file_name = get_file_name(node.name, ext="avi")
		self.video_object = cv2.VideoWriter(save_path+self.file_name, cv2.VideoWriter_fourcc(*'MJPG'),fps=fps, size=size)

	def run(self, args=None):
		while self.node.record:
			self.video_object.write(self.node.webrtc_image)
		self.video_object.release()