#!/usr/bin/env python

#Implementation of Linear Quadratic Regulator Agorithm on osrf car_demo
#Author: Shivam Sood
#Date: 19/09/2020
 
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry
from scipy import spatial
import numpy as np
import scipy.linalg as la
import math
import time
import tf

path_arr = list()
path_cords = list()
L = 3
prius_pos = []
steer_cmd = 0
yaw = 0
velocity = 0
prev_err = 0
prev_theta_err = 0

target_vel = 3
vx, vy = 0,0
kp =1 		#Proportional Term
ki = 0.000034   #Integral Term
kd = 0.002      #Derivative Term (Note that delta_t is considered to be already multiplied with them hence the small values)
error,sum_error,diff_error =0,0,0 

def vel_callback(Odometry):
    global vx,vy,error,sum_error,diff_error
    vx = Odometry.twist.twist.linear.x      
    vy = Odometry.twist.twist.linear.y     #Get the x,y components of velocity of prius
    vel = math.sqrt(vx**2 + vy**2)       
    prev_error = error                     #For the  derivative term
    error = target_vel - vel
    sum_error += error
    dif_error = error-prev_error
    print(vel)

def create_kinematic_model():       #Actually a modified dynamic model
    global velocity

    A = np.matrix(np.zeros((4, 4)))
    A[0, 1] = 1.0
    A[1, 1] = -(272.79)/(velocity+math.pow(10,-7)) 
    A[1, 2] = 272.79
    A[2, 3] = 1.0
    A[3, 3] = -293.88/(velocity+math.pow(10,-7))

    B = np.matrix(np.zeros((4, 1)))
    B[1, 0] = 136.39
    B[3, 0] = 126.13

    return A,B

def create_cost_matrix():   #LQR tuning params
    Q = np.zeros((4,4))
    R = 0.001
    Q[0,0] = 5000
    Q[2,2] = 20000000
    return Q,R

def lqr_optimal_matrix(A,B,Q,R):
    global K
    P = la.solve_continuous_are(A,B,Q,R)
    K =  np.dot(np.dot(R,np.transpose(B)),P)
    w,v = np.linalg.eig(A-np.dot(B,K))

    #P = Q
    #At = np.transpose(A)
    #Bt = np.transpose(B)

    #for i in range(30):
    #    T1 = np.dot(np.dot(At,P),A)
    #    T2 = np.dot(np.dot(At,P),B)
    #    T3 = np.linalg.inv(np.dot(np.dot(Bt,P),B)+R)
    #    T4 = np.dot(np.dot(Bt,P),A)
    #    P =  Q+T1-np.dot(np.dot(T2,T3),T4)
        #print P

    
    
    #T5 = np.linalg.inv(np.dot(np.dot(Bt,P),B)+R)
    #T6 = np.dot(np.dot(Bt,P),A)
    #K =  np.dot(T5,T6)
    #w,v = np.linalg.eig(A-np.dot(B,K))
    
    #print w
    #print K        #TODO - Solution of Algebraic Ricatti eqn without scipy
    return K


def quaternion_euler(o_x,o_y,o_z,o_w):
    quaternion = (o_x,o_y,o_z,o_w)
    euler = tf.transformations.euler_from_quaternion(quaternion)	#Uses tf library to compute roll,pitch and yaw from quaternions
    yaw = euler[2]
    return yaw

def pos_orient_velocity_callback(Odometry):							#Callback function for base_pose_ground truth
    global prius_pos , yaw , velocity
    prius_x = Odometry.pose.pose.position.x
    prius_y = Odometry.pose.pose.position.y
    prius_pos = [prius_x,prius_y]
    
    o_x = Odometry.pose.pose.orientation.x
    o_y = Odometry.pose.pose.orientation.y
    o_z = Odometry.pose.pose.orientation.z
    o_w = Odometry.pose.pose.orientation.w
    yaw = quaternion_euler(o_x,o_y,o_z,o_w)

    v_x = Odometry.twist.twist.linear.x
    v_y = Odometry.twist.twist.linear.y
    velocity = math.sqrt(math.pow(v_x,2)+math.pow(v_y,2))

def find_cross_track_error(path_cords,prius_pos,yaw):                               #Using Scipy and Tree structure to quickly find closest point from path co-ordinates
    distance,index = spatial.KDTree(path_cords).query(prius_pos)
    nearest_pt = path_cords[index]
    return [distance,index,nearest_pt]

def find_dirn_ct_error(prius_pos,cross_track_err,yaw):				#To assign direction to cross track error (+1 for left side and -1 for right side)
    y_tf = -1*(cross_track_err[2][0]-prius_pos[0])*math.sin(yaw)+(cross_track_err[2][1]-prius_pos[1])*math.cos(yaw)		#Transforms coordinates of point such that car is at origin 
    																													#and pointed along x axis wrt to that point
    if y_tf > 0 :		#positive y-axis(left)
        dirn_err = 1
    else :				#negative y-axis(right)
        dirn_err = -1

    dirn_ct_error = dirn_err*cross_track_err[0]
    #print dirn_ct_error
    return dirn_ct_error

	
def calculate_theta_error(yaw,path_cords,cross_track_err):     #To calculate orientation error
    epsilon = math.pow(10,-7)   #To prevent division by Zero

    i = cross_track_err[1]
    x1 = cross_track_err[2][0]
    y1 = cross_track_err[2][1]
    x2 = path_cords[i+1][0]
    y2 = path_cords[i+1][1]
    x3 = path_cords[i-1][0]
    y3 = path_cords[i-1][1]
    
    if i == 0 :
        theta_path = math.atan2((y2-y1),(x2-x1+epsilon))    #Tangent of path(using p and p+1 point)
    else :
        theta_path = math.atan2((y2-y3),(x2-x3+epsilon))   #Tangent of path (using p+1 and p-1 point)

    theta_prius = yaw

    theta_error = -1*(theta_prius - theta_path)            #Heading error , must be of opposite dirn than that of angle calculated using cross track error 
    return theta_error

def calculate_state_matrix(e,th_e,pe,pth_e):

    global prev_err,prev_theta_err
    x = np.matrix(np.zeros((4, 1)))

    x[0, 0] = e
    x[1, 0] = (e - pe)
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e)

    prev_err = e
    prev_theta_err = th_e

    #print (e,th_e,pe,pth_e)
    return x

def calculate_steer_angle(X,K):
    steer_angle = (1*np.dot(K,X))[0, 0]
    steer_angle = 13*math.tanh(steer_angle)
    
    #print steer_angle
    return steer_angle

def path_callback(Path):											#Callback function for astroid_path
    global path_arr,path_cords,prius_pos,steer_cmd,yaw,velocity
    global prev_err,prev_theta_err
    path_arr = Path.poses
    for i in range(len(path_arr)):
    	p_x = path_arr[i].pose.position.x
    	p_y = path_arr[i].pose.position.y
    	path_cords.append([p_x,p_y])
    
    A,B = create_kinematic_model()
    Q,R = create_cost_matrix()
    K = lqr_optimal_matrix(A,B,Q,R)

    cross_track_err = find_cross_track_error(path_cords,prius_pos,yaw)
    error = find_dirn_ct_error(prius_pos,cross_track_err,yaw)
    theta_error = calculate_theta_error(yaw,path_cords,cross_track_err)

    X = calculate_state_matrix(error,theta_error,prev_err,prev_theta_err)

    steer_cmd = calculate_steer_angle(X,K)
    path_cords =[]													#Empty list to avoid growth of list after each callback 


def lqr_ctrl():
    global path_arr
    global path_cords
    global prius_pos , yaw
    global steer_cmd

    pub = rospy.Publisher('prius', Control, queue_size=10)
    rospy.init_node('lqr_ctrl', anonymous=True)

    rospy.Subscriber('base_pose_ground_truth',Odometry, pos_orient_velocity_callback)

    rospy.Subscriber('astroid_path',Path, path_callback)

    rospy.Subscriber('/base_pose_ground_truth' , Odometry, vel_callback)

    rate = rospy.Rate(20)                	
    ctrl = Control()

    while not rospy.is_shutdown():
    	ctrl.steer = steer_cmd
	val = kp*error+ki*sum_error+kd*diff_error										
        if val>1:
            val = 1
        if val<0:
            val = 0
        ctrl.throttle =val
        print(val)
    	pub.publish(ctrl)
    	rate.sleep()
        
if __name__ == '__main__':
    try:
        lqr_ctrl()
    except rospy.ROSInterruptException:
        pass
