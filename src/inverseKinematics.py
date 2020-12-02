import numpy as np
from std_msgs.msg import Float64
import rospy

PI = 3.14159

class kinematics :
    def __init__(self) :
        self.joint1_pub = rospy.Publisher('/thorvald_arm/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/thorvald_arm/joint2_position_controller/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/thorvald_arm/joint3_position_controller/command', Float64, queue_size=10)

        rospy.init_node('kinematicSolver', anonymous=True)
        rate = rospy.Rate(10)

    def publish_angles(self,theta_1,theta_2,theta_3) :
        self.joint1_pub.publish(theta_1)
        self.joint2_pub.publish(theta_2)
        self.joint3_pub.publish(theta_3)

    def inverseKinematics(self,x,y,L1 = 0.32,L2= 0.32,L3 = 0.15):
    # https://www.daslhub.org/unlv/wiki/doku.php?id=2_link_kinematics

        try :
            # Translate from nozzle end point to nozzle joint
            y += L3

            cos_theta_2 = (x**2 + y**2 - L1**2 + L2**2)/ 2*L1*L2
            theta_2 = np.arccos(cos_theta_2)
            theta_1 = np.arctan2(y/x)  - np.arctan2( (L2*np.sin(theta_2))/(L1 + L2*np.cos(theta_2) ) ) 

            # Theta 3 keeps the nozzle in the same pose as the base frame
            theta_3 = -theta_1 -theta_2

            return theta_1, theta_2, theta_3

        except :
            print("Kinematic Error")

