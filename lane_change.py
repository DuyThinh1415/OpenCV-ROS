from os import terminal_size
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

public_velo = rospy.Publisher("/cmd_vel",Twist, queue_size=5)
my_Velo = Twist()


clock_count = 0;

#157 183 340


class List_Point:
	def __init__(self):
		self.row = np.array(range(20))
		self.col = np.array(range(20))
		self.trust = np.array(range(20))
		self.size=0

def callbackFunction(image):

	global clock_count

	delta_d = 0.5
	V_linear = 0.1
	V_angular = 0.15
	alpha = 15

	V_n_angular = V_angular
	V_angular = V_angular*360/(2*math.pi)
	print("V angular ",V_angular)

	clk1 = 20*alpha/V_angular

	print("clk1 ",clk1)

	alpha = (alpha*2*math.pi)/360

	print("alpha ",alpha)

	#============= check condition ===========================

	b = V_linear/V_angular
	print("b ",b)
	d1=((180*b)/(math.pi))*(1-math.cos(alpha))
	print("d1 ",d1)
	if (d1 > delta_d/2):
		print(" => NO <= ",d1)

	
	clk2 = (delta_d - (360*b/math.pi)*(1-math.cos(alpha)))*20/(V_linear*math.sin(alpha))
	print("clk2 ",clk2)
	clk1/=2
	clk2/=2
	#=============     process     ===========================

	if clock_count < clk1:
		my_Velo.linear.x=V_linear
		my_Velo.angular.z=-1*V_n_angular
		print("A->B : ",clock_count)
	elif clock_count < clk1+clk2:
		my_Velo.linear.x=V_linear
		my_Velo.angular.z=0
		print("B->C : ",clock_count)
	elif clock_count < 2*clk1+clk2:
		my_Velo.linear.x=V_linear
		my_Velo.angular.z=V_n_angular
		print("C->A : ",clock_count)
	else:
		my_Velo.linear.x=V_linear
		my_Velo.angular.z=0

	print("CLK1 : ",clk1," CLK2 : ",clk2," \n ========================================== \n ")

	clock_count+=1 
	print(" => ",clock_count," <=");
	public_velo.publish(my_Velo)


while not rospy.is_shutdown():
	rospy.init_node("topic_receiver",anonymous=True)
	lis_scan = rospy.Subscriber("/camera/rgb/image_raw", Image, callback=callbackFunction)
	rospy.spin()