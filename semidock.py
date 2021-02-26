#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Byte, Bool, Int16
from geometry_msgs.msg import Twist
from james_docking.msg import dockAction, dockFeedback, dockResult, sensor_state
import time
class jamesDocking(object):

    _dockingfb = dockFeedback()
    _result = dockResult()

    def __init__(self):
        self._dockServer = actionlib.SimpleActionServer('/try_dock', dockAction, self.feedback_callback, False )
        self._dockServer.start()
        self._velPublisher = rospy.Publisher('/base_controller/cmd_vel', Twist, queue_size=10)
        self._relay_pub = rospy.Publisher('/relay', Bool, queue_size=10)

        self._irState = sensor_state()
        self._velocity = Twist()
        self._relay_ON = Bool()
        self._dockState = Byte()
        self._bumperState = Bool()
        self._irSubscriber = rospy.Subscriber('/dockdata_T', Int16, self.cb_getIR)
        self._bumperSubscriber = rospy.Subscriber('/bumper_state', Bool, self.cb_getBUMPER)

        # initialize velocity
        self._velocity.linear.x = 0
        self._velocity.linear.y = 0
        self._velocity.linear.z = 0
        self._velocity.angular.x = 0
        self._velocity.angular.y = 0
        self._velocity.angular.z = 0

        # initialize relay
        self._relay_ON = False
        

        rospy.loginfo('Server start...')

    def feedback_callback(self, goal):
        # this callback is called when the action server is called.
        # this is the function that runs auto docking.
        self._success = False
        serverCalled = goal.order

        if serverCalled == True:            
            self._dockState = 0
            rospy.loginfo('Goal arrived...')  

        while not self._success:

            if self._dockServer.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                self._linear_vel(0)
                self._angular_vel_ck(0)
                self._velPublisher.publish(self._velocity)
                self._dockServer.set_preempted()
                break            

            if self._dockState == 0:
                rospy.loginfo('Start docking process...')
                self._dockingfb.sequence = "Start docking process..."
                self._dockServer.publish_feedback(self._dockingfb)
                
                if int(bin(self.dockdata & 0b001001000), 2) >= 0b000001000 :
                    self._angular_vel_ck(1.5)
                
                else:
                    self._angular_vel_ack(1.5)
                
                self._dockState = 1

            elif self._dockState == 1:
              
                if int(bin(self.dockdata & 0b101111101), 2) >= 0b000000001 :          
                    self._angular_vel_ck(0)
                    self._velPublisher.publish(self._velocity)
                    self._dockingfb.sequence = 'IR transmitter found!...'
                    self._dockServer.publish_feedback(self._dockingfb)                         

                    if int(bin(self.dockdata & 0b000111000), 2) >= 0b000001000 :
                        self._dockState = 5
                    else :
                        self._dockState = 2
                
                self._velPublisher.publish(self._velocity)
                
            elif self._dockState == 2:
                if int(bin(self.dockdata & 0b000111000), 2) >= 0b000001000 :
                    self._dockingfb.sequence = 'Center line found...'
                    self._dockServer.publish_feedback(self._dockingfb)                         
                    self._linear_vel(0)
                    self._velPublisher.publish(self._velocity)
                    self._dockState = 5
                
                elif int(bin(self.dockdata & 0b111000000), 2) >= 0b001000000 :
                        self._dockingfb.sequence = 'Rotating to the center line...'
                        self._dockServer.publish_feedback(self._dockingfb)                         
                        self._linear_vel(0)
                        self._velPublisher.publish(self._velocity)
                        self._dockState = 3

                elif int(bin(self.dockdata & 0b000000111), 2) >= 0b000000010 :
                        self._dockingfb.sequence = 'Rotating to the center line...'
                        self._dockServer.publish_feedback(self._dockingfb)                         
                        self._linear_vel(0)
                        self._velPublisher.publish(self._velocity)
                        self._dockState = 4

                self._linear_vel(1)
                self._angular_vel_ck(0)
                self._velPublisher.publish(self._velocity)

            elif self._dockState == 3:
                self._linear_vel(0)
                self._angular_vel_ck(3)
                self._velPublisher.publish(self._velocity)
                
                if int(bin(self.dockdata & 0b000111001), 2) >= 0b000000001 :
                    self._dockState = 5
                    self._dockingfb.sequence = 'Get ready to dock...'
                    self._dockServer.publish_feedback(self._dockingfb)                         
                    
            elif self._dockState == 4:
                self._linear_vel(0)
                self._angular_vel_ack(3)
                self._velPublisher.publish(self._velocity)
                
                if int(bin(self.dockdata & 0b100111000), 2) >= 0b000001000 :
                    self._dockState = 5
                    self._dockingfb.sequence = 'Get ready to dock...'
                    self._dockServer.publish_feedback(self._dockingfb)                         

            elif self._dockState == 5:
                    self._velocity.angular.z = 0 
                    self._velPublisher.publish(self._velocity)
                    self._dockingfb.sequence = "Initiate docking..."
                    self._dockServer.publish_feedback(self._dockingfb)
                    self._dockState = 6

            elif self._dockState == 6:

                if (self._bumperState.data == 1):
                    self._linear_vel(0)
                    self._angular_vel_ck(0)
                    self._success = True

                if int(bin(self.dockdata & 0b000010000), 2) == 0b000010000 : #   ~|~|~  ~|C|~  ~|~|~
                    self._linear_vel(1)
                    self.angular_z = 0
                    if int(bin(self.dockdata & 0b000100000), 2) == 0b000100000 : #   ~|~|~  L|C|~  ~|~|~
                        self.angular_z += -0.1
                    if int(bin(self.dockdata & 0b000001000), 2) == 0b000001000 : #   ~|~|~  ~|C|R  ~|~|~
                        self.angular_z += 0.1
                    
                    self._velocity.angular.z = self.angular_z

                elif int(bin(self.dockdata & 0b10000001), 2) >= 0b000000001 : #   L|~|~  ~|~|~  ~|~|R
                    self._linear_vel(1)
                    self.angular__z = 0

                    if int(bin(self.dockdata & 0b000000001), 2) == 0b000000001 : #   ~|~|~  ~|~|~  ~|~|R
                        self.angular__z += -0.1

                    if int(bin(self.dockdata & 0b100000000), 2) == 0b100000000 : #   L|~|~  ~|~|~  ~|~|~
                        self.angular__z += 0.1

                    self._velocity.angular.z = self.angular__z

                elif int(bin(self.dockdata & 0b000100000), 2) >= 0b000000001 : #   ~|~|~  L|~|~  ~|~|~
                    self._linear_vel(1)
                    self._angular_vel_ack(1)

                elif int(bin(self.dockdata & 0b000001000), 2) >= 0b000001000 : #   ~|~|~  ~|~|R  ~|~|~
                    self._linear_vel(1)
                    self._angular_vel_ck(1)

                elif int(bin(self.dockdata & 0b001000000), 2) >= 0b001000000 : #   ~|~|R  ~|~|~  ~|~|~
                    self._linear_vel(1)
                    self._angular_vel_ck(2)

                elif int(bin(self.dockdata & 0b000000100), 2) >= 0b000000100 : #   ~|~|~  ~|~|~  L|~|~
                    self._linear_vel(1)
                    self._angular_vel_ack(2)

                else :
                    self._linear_vel(1)
                    self._angular_vel_ck(0)

                self._velPublisher.publish(self._velocity)

            if self._success == True:
                self._linear_vel(0)
                self._angular_vel_ck(0)
                self._velPublisher.publish(self._velocity)
                if int(bin(self.dockdata & 0b000010000), 2) == 0b000010000 :
                    self._relay_pub.publish(self._relay_ON)
                    self._result.consequence = self._success
                    rospy.loginfo('Successfully docked!')
                    self._dockingfb.sequence = "Successfully docked!"
                    self._dockServer.publish_feedback(self._dockingfb)
                    self._dockServer.set_succeeded(self._result)
                else :
                    self._result.consequence = self._success
                    rospy.loginfo('docked fail...')
                    self._dockingfb.sequence = "docked fail..."
                    self._dockServer.publish_feedback(self._dockingfb)
                    self._dockServer.set_succeeded(self._result)
        
        
            rate.sleep()
            

    def cb_getIR(self, msg):
        self._dockdata = msg
        self.dockdata = self._dockdata.data

    def cb_getBUMPER(self, msg):
        self._bumperState = msg

    def publish_vel(self):
        self._velPublisher.publish(self._velocity)
    
    def _linear_vel(self, l_multiple_num):
        self._velocity.linear.x = l_multiple_num * -0.05

    def _angular_vel_ck(self, a_multiple_num):
        self._velocity.angular.z = a_multiple_num * 0.1

    def _angular_vel_ack(self, a_multiple_num):
        self._velocity.angular.z = a_multiple_num * -0.1 


if __name__ == '__main__':
    rospy.init_node('james_docking')
    rate = rospy.Rate(3000)
    jamesDocking()