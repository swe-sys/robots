#!/usr/bin/env python3
from __future__ import division
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
from math import atan, atan2, pi, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from kalman.msg import Trilateration
from trilateration import varA, varB, varC, landmarkA, landmarkB, landmarkC

# Define global variables
pose = [0.0, 0.0, 0.0]
prev_pose = [0.0, 0.0, 0.0]
input_sys = np.array([0.0, 0.0])

K_samp = 0.7
tt = 0
predicted_pose = np.array([[0.0], [0.0], [0.0]])
estimated_pose = np.array([[0.0], [0.0], [0.0]])
our_predicted_pose = np.array([[0.0], [0.0], [0.0]]) 
pose_list = [] 
# System noise related
noisy_pose = np.array([0.0, 0.0, 0.0])
varTHETA, varX, varY = 0.01, 0.05, 0.05
# varTHETA, varX, varY = 0.0, 0.0, 0.0

Q = np.diag([varX, varY, varTHETA])  # Process Noise
R = np.diag([varA, varB, varC])  # Imported from trilateration
P = np.diag([0.5, 0.5, 0.5])  # Some reasonable initial values
F = np.eye(3)  # System matrix for discretized unicycle is Identity
# get_current_H(pose, landmarkA, landmarkB, landmarkC)  ## H has to be calculated on the fly
H = np.zeros((3, 2))
#G = np.zeros((3, 2))
I = np.eye(3)  # identity matrix

distanceLandmarkA = 10.0
distanceLandmarkB = 10.0
distanceLandmarkC = 10.0

FILTER_ORDER = 5  # Change filter settings
i = 0
filter_a = [0 for i in range(FILTER_ORDER)]
filter_b = [0 for i in range(FILTER_ORDER)]
filter_c = [0 for i in range(FILTER_ORDER)]

idxA = 0
idxB = 0
idxC = 0
theta = 0

odoo = Odometry()
depub = ''

estimated_angle = 0.0 
predicted_angle = 0.0

def heading_from_quaternion(x, y, z, w):
    ang_1 = 2*(w*z + x*y)
    ang_2 = 1-2*(y**2 + z**2)
    return atan2(ang_1,ang_2) % (2*pi)


def get_current_H(pose, lA, lB, lC):
    # Calculate the linearized measurement matrix H(k+1|k) at the current robot pose

    # x and y co-ordinates of landmarks
    # # print(pose)
    xA, yA = lA
    xB, yB = lB
    xC, yC = lC
    # current robot pose
    # # print(pose.shape)
    x, y = pose[0][0], pose[1][0]
    # # print(x, y, 'xy')
    # Linearized H
    H = [[x-xA, y-yA, 0], [x-xB, y-yB, 0], [x-xC, y-yC, 0]]
    # H = np.array(H).reshape(3, 3)
    # # print(H,'hjk')

    return 2 * np.array(H)


def sq_dist(p1, p2):
    # Given a pair of points the function returns euclidean distance
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)#**(0.5)


def predict_state(estimated_pose):
    # System evolution
    global noisy_pose, pose, input_sys, F, P, our_predicted_pose, estimated_angle, predicted_angle 
    input_sys.shape = (2, 1)
    # predicted_pose = (np.matmul(F, noisy_pose)) + (np.matmul(G, input_sys))
    v = 0.62*input_sys[0] 
    w = 0.65*input_sys[1] 
    print(w,'omega') 
    predicted_pose = estimated_pose + (1/5)*np.array([v*cos(estimated_angle), v*sin(estimated_angle), w])
    predicted_angle = estimated_angle + (1/5)*w 
    print(predicted_pose[2], 'pp')
    # # # print(predicted_pose1,predicted_pose,'dfgf')
    return predicted_pose #np.matrix(pose).T #


def predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C):
    # Predicts the measurement (d1, d2, d3) given the current position of the robot
    d1 = sq_dist(predicted_pose, landmark_A)
    d2 = sq_dist(predicted_pose, landmark_B)
    d3 = sq_dist(predicted_pose, landmark_C)
    measurement = [d1, d2, d3]
    print(measurement,'MM')
    measurement = np.array(measurement).reshape(3, 1)
    return measurement


def callback2(data):
    global noisy_pose, varX, varY, varTHETA
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    noise = [np.random.normal(0, varX), np.random.normal(
        0, varY), np.random.normal(0, varTHETA)]
    noisy_pose = np.array([data.pose.pose.position.x + noise[0], data.pose.pose.position.y +
                          noise[1], euler_from_quaternion([x, y, z, w])[2] + noise[2]]).reshape(3, 1)

def callback_vicon(data):
    pos_x = data.transform.translation.x
    pos_y = data.transform.translation.y
    orientation_q = data.transform.rotation
    heading = heading_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    noisy_pose = np.array([pos_x,pos_y, heading]).reshape(3,1)

    
def get_waypoint(t):
    global K_samp  # defined at line 17
    R = 5  # radius
    m = R*cos(t*K_samp)
    n = R*sin(t*K_samp)
    return [m, n]

def callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, theta, pose
    global P, Q, R, F
    global estimated_pose, noisy_pose, our_predicted_pose, estimated_angle, predicted_angle, pose_list  

    lA = data.landmarkA
    lB = data.landmarkB
    lC = data.landmarkC

    #######################################################
    # FILTERING VALUES
    #######################################################
    # Add value into r buffer at indices idxA, idxB, idxC
    filter_a[idxA] = lA.distance
    filter_b[idxB] = lB.distance
    filter_c[idxC] = lC.distance
    # Increment indexes
    idxA += 1
    idxB += 1
    idxC += 1

    # wrap around the indices if buffer full
    if idxA >= FILTER_ORDER:
        idxA = 0
    if idxB >= FILTER_ORDER:
        idxB = 0
    if idxC >= FILTER_ORDER:
        idxC = 0

    # Calculate filtered measurements (d1, d2, d3)
    x1, y1 = 7, 7
    x2, y2 = -7, -7
    x3, y3 = 7, -7
    d1, d2, d3 = sum(filter_a)/FILTER_ORDER, sum(filter_b) / FILTER_ORDER, sum(filter_c)/FILTER_ORDER

    Y_measured = np.matrix([[d1**2], [d2**2], [d3**2]])  # z vector

    # EXTENDED KALMAN FILTER CODE GOES BELOW

    # Prediction:
    #    you may use the function 'predict_state()' defined above as a substitute for prediction
    # x(k+1|k)
    predicted_pose = predict_state(estimated_pose)
    print(predicted_pose[2], predicted_angle, 'pp')
    # # # print(predicted_pose, '1')

    # Covariance update:
    #    use the linearized state space matrix F to calculate the predicted covariance matrix
    # P(k+1|k)
    predicted_state_covariance = np.matmul(np.matmul(F, P), F.transpose()) + Q #P(k/k-1)

    # Get measurement residual:
    #    difference between y_measured and y_predicted of the (k+1)^th time instant
    predicted_measurement = predict_measurement(predicted_pose, landmarkA, landmarkB, landmarkC) # h(x)

    # print(Y_measured,'Filter')
    # print(predicted_measurement, 'Predicted')

    residual = Y_measured - predicted_measurement

    # Kalman gain calculation:
    #    use the linearized measurement matrix 'H(k+1|k)' to compute the kalman gain
    # W(k+1)
    H = get_current_H(np.array(estimated_pose), landmarkA, landmarkB, landmarkC)
    # # print(H, 'H')
    PH_T = np.matmul(predicted_state_covariance, np.matrix(H).transpose())
    # # print(PH_T, 'PH')
    S = np.matmul(H, PH_T) + R
    # S = np.array([[np.squeeze(np.asarray(S[j][i])) for i in range(3)] for j in range(3)]) 
    S = S.tolist()[:] 
    # # print(S, 'S')   
    S_inv = np.linalg.inv(S)   
    # # print(S_inv, 'sd')
    filter_gain = np.matmul(PH_T, S_inv)
    # # print(filter_gain, 'rt')

    # Update state with kalman gain:
    #    correct the predicted state using the measurement residual
    # x(k+1|k+1)
    print(predicted_pose[2], estimated_pose[2],'pp ep') 
    pose_list.append(estimated_pose[:,0].tolist()) 
    if len(pose_list)>20: 
        pose_list.pop(0) 
    prev_estimated_pose = np.mean(np.array(pose_list),axis=0) #x(k|k)-averaged 
    # prev_estimated_pose = estimated_pose #x(k|k)  
    estimated_pose = predicted_pose + np.matmul(filter_gain, residual) #x(k+1|k+1) 
    # estimated_pose[2] = atan2(estimated_pose[1]-prev_estimated_pose[1],estimated_pose[0]-prev_estimated_pose[0])    
    print(predicted_pose[2], estimated_pose[2],'pp ep') 
    lam = 0.01 # noisy_pose[2] #(1-lam)*predicted_angle + lam*
    estimated_angle = atan2(estimated_pose[1]-prev_estimated_pose[1],estimated_pose[0]-prev_estimated_pose[0])   
    estimated_pose[2] = estimated_angle 
    print(estimated_angle,'chk') 

    x = estimated_pose[0]
    y = estimated_pose[1]
    theta = estimated_pose[2]
    # Update covariance matrix
    #
    # P(k+1|k+1)
    KH = np.matmul(filter_gain, H)
    predicted_state_covariance = np.matmul(predicted_state_covariance, (I - KH))   

    # Do not modify the code below
    # Send an Odometry message for visualization (vis.py)
    odoo.pose.pose.position.x = x
    odoo.pose.pose.position.y = y
    quaternion_val = quaternion_from_euler(0, 0, theta)
    odoo.pose.pose.orientation.x = quaternion_val[0]
    odoo.pose.pose.orientation.y = quaternion_val[1]
    odoo.pose.pose.orientation.z = quaternion_val[2]
    odoo.pose.pose.orientation.w = quaternion_val[3]
    depub.publish(odoo)

    # print("x:{}, \ty:{}, \ttheta:{}\n".format(x, y, theta*(180/pi)))


# This is where we will write code for trajectory following
def control_loop():
    global pose, depub, input_sys, estimated_pose, noisy_pose, our_predicted_pose, estimated_angle

    rospy.init_node('controller_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    #rospy.Subscriber('/odom', Odometry, callback2)
    rospy.Subscriber('/vicon/fb5_8/fb5_8', TransformStamped, callback_vicon)
    depub = rospy.Publisher('/odom2', Odometry, queue_size=10)

    pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)
    pubep = rospy.Publisher('/bot_0/estimatedpose', String, queue_size=10)

    # Setting the rate for loop execution
    rate = rospy.Rate(5)

    # Twist values to move the robot
    timer = 0
    #updated_pose = estimated_pose

    while not rospy.is_shutdown():
        wp = get_waypoint(timer)
        distance = sqrt(pow((wp[0]-estimated_pose[0]), 2) + pow((wp[1]-estimated_pose[1]), 2))       
        # head_angle = 2.0*estimated_pose[2]
        head_angle = estimated_angle #estimated_pose[2] 
        ref_angle = atan2(wp[1] - estimated_pose[1], wp[0] - estimated_pose[0])

        if ref_angle<0 and head_angle>0 and abs(head_angle)>pi/2: 
            angle = 2*pi + ref_angle - head_angle 
        elif ref_angle>0 and head_angle<0 and abs(head_angle)>pi/2: 
            angle = ref_angle - (2*pi + head_angle) 
        else: 
            angle = ref_angle - head_angle 
        
        #Wrapping 
        xwrap = angle%(2*pi)
        if abs (xwrap) > pi: 
            xwrap = xwrap - 2*pi * np.sign(xwrap)
       
        # print("apple = ", ref_angle*180/np.pi, head_angle*180/np.pi, angle*180/np.pi)
        # print("W0 {} W1 {}".format(wp[0], wp[1]))
        # print("p0 {} p1 {} p2 {}".format(estimated_pose[0], estimated_pose[1], estimated_pose[2]))
        # print("np0 {} np1 {} np2 {}".format(noisy_pose[0], noisy_pose[1], noisy_pose[2]))
        # print("up0 {} up1 {} up2 {}".format(estimated_pose[0], estimated_pose[1], head_angle))
        velocity_msg = Twist() 
        velocity_msg.linear.x = 0.09*distance
        velocity_msg.angular.z = 0.5*(angle)
        input_sys[0] = velocity_msg.linear.x
        input_sys[1] = velocity_msg.angular.z
        timer = timer + 0.004 * pi

        # If robot has reached the current waypoint
        # Sample a new waypoint
        # Apply proportional control to reach the waypoint
        ####
        pub.publish(velocity_msg)
        pubw.publish(str(wp))
        # pubep.publish(str([estimated_pose.tolist()[i][0] for i in range(2)])) 
        pubep.publish(str(noisy_pose.tolist())) 
        # # print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
