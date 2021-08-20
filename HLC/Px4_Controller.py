
import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from math import sqrt
from time import sleep

class FLIGHT_CONTROLLER:
	def __init__(self):
		#DATA
		self.gps_lat = 0
		self.gps_long = 0
		self.curr_x = 0
		self.curr_y = 0
		self.curr_z = 0
		self.delta = 0.25
		self.waypoint_number = 0

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		#self.gps_subscriber =  rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
		#self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		rospy.loginfo('INIT')
		
		
	#MODE SETUP
	def toggle_arm(self, arm_bool):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			self.arm_service(arm_bool)
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):
		self.gps_subscriber
		t_lat = self.gps_lat
		t_long = self.gps_long
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0.0, 0.0, t_lat, t_long, t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
			
	def land(self, l_alt):
		self.gps_subscriber
		l_lat = self.gps_lat
		l_long = self.gps_long
		rospy.wait_for_service('/mavros/cmd/land')
		
		try:
			self.land_service(0.0, 0.0, l_lat, l_long, l_alt)
			rospy.loginfo("LANDING")
			
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
			
	def set_offboard_mode(self):

		rate=rospy.Rate(20)
		#print('OFF')

		rospy.wait_for_service('/mavros/set_mode')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 3.37



		for i in range(50):
			self.publish_pose.publish(PS)
			rate.sleep()
		try:
			self.flight_mode_service(0, 'OFFBOARD')
			rospy.loginfo('OFFBOARD')


		except rospy.ServiceException as e:
			rospy.loginfo("OFFBOARD Mode could not be set: " %e)
		
		'''rospy.wait_for_service('/mavros/set_mode')
		try:
			self.flight_mode_service(0, 'OFFBOARD')
			rospy.loginfo('OFFBOARD')
			
		except rospy.ServiceExceptions as e:
			rospy.loginfo("OFFBOARD Mode could not be set: " %e)'''
			
	#CALLBACKS
	
	def gps_callback(self, data):
		self.gps_lat = data.latitude
		self.gps_long = data.longitude
		
	def get_pose(self, location_data):
		
		self.curr_x = location_data.pose.position.x
		self.curr_y = location_data.pose.position.y
		self.curr_z = location_data.pose.position.z
		
	#PUBLISHERS
	 
	def set_pose(self, set_x, set_y, set_z):
		update_rate = rospy.Rate(20)
		PS = PoseStamped()
		
		PS.pose.position.x = set_x
		PS.pose.position.y = set_y
		PS.pose.position.z = set_z
		
		distance = sqrt((set_x - self.curr_x)**2 + (set_y - self.curr_y)**2 + (set_y - self.curr_z)**2)
		
		while distance > self.delta:
			
			self.publish_pose.publish(PS)
			self.get_pose_subscriber
			distance = sqrt((set_x - self.curr_x)**2 + (set_y - self.curr_y)**2 + (set_z - self.curr_z)**2)
			update_rate.sleep()
		
		self.waypoint_number = self.waypoint_number + 1
		rospy.loginfo('WAYPOINT REACHED: ' + str(self.waypoint_number))
		
		
	#MISSION CONTROL
	
	def mission_control(self):
		self.set_offboard_mode()
		self.toggle_arm(True)
		self.takeoff(3.0)
		sleep(5.0)
		#self.set_offboard_mode()
		self.set_pose(0.0, 0.0, 3.0)
		self.set_pose(2.0, 0.0, 3.5)
		self.set_pose(2.0, 2.5, 3.5)
		
if __name__ == '__main__':
	try:
		iris_controller = FLIGHT_CONTROLLER()
		iris_controller.mission_control()
		rospy.spin()
			
	except rospy.ROSInterruptException:
		pass

	
