#!/usr/bin/env python
'''

Created March, 2017

@author: Peter Heim

  arm_driver.py - gateway to Arduino based arm controller
  Copyright (c) 2011 Peter Heim.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rospy
import tf
import math
from math import sin, cos, pi, radians, degrees
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import Float64, Float32
from dynamixel_msgs.msg import JointState as JointStateDY
from sensor_msgs.msg import JointState
from SerialDataGateway import SerialDataGateway

class Arm_Driver(object):
        '''
        Helper class for communicating with an arduino board over serial port
        '''



        def _HandleReceivedLine(self,  line):
                self._Counter = self._Counter + 1
                #rospy.logwarn(str(self._Counter) + " " + line)
                #if (self._Counter % 50 == 0):
                self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

                if (len(line) > 0):
                        lineParts = line.split('\t')                 
                        if (lineParts[0] == 'j1'):
                                self._Broadcast_Left_Lift_Joint(lineParts)
                                return
                        if (lineParts[0] == 'j2'):
                                self._Broadcast_Left_Rotate_Joint(lineParts)
                                return                    
                        if (lineParts[0] == 'j3'):
                                self._Broadcast_Right_Lift_Joint(lineParts)
                                return
                        if (lineParts[0] == 'j4'):
                                self._Broadcast_Right_Rotate_Joint(lineParts)
                                return
                        if (lineParts[0] == 'j5'):
                                self._Broadcast_left_gripper_Joint(lineParts)
                                return
                        if (lineParts[0] == 'j6'):
                                self._Broadcast_Right_gripper_Joint(lineParts)
                                return



        def _Broadcast_Left_Lift_Joint(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(P1)
                if (partsCount  < 2):
                        pass
                try:
                    P1 = (0 + (radians(float(lineParts[1]))))-1.05
                    Joint_State = JointStateDY()
                    Joint_State.name = "left_lift_joint"
                    Joint_State.current_pos = P1
                    Joint_State.header.stamp = rospy.Time.now()
                    self._P2_JointPublisher.publish(Joint_State)
                    #rospy.logwarn(Joint_State)

                except:
                    rospy.logwarn("Unexpected error:left_lift_joint" + str(sys.exc_info()[0]))



        def _Broadcast_Left_Rotate_Joint(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 2):
                        pass
                try:
                        P1 = (0 - (radians(float(lineParts[1])))) +1.57
                        Joint_State = JointStateDY()
                        Joint_State.name = "left_rotate_joint"
                        Joint_State.current_pos = P1
                        
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P3_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:left_rotate_joint" + str(sys.exc_info()[0]))

       


        def _Broadcast_Right_Lift_Joint(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 2):
                        pass
                try:
                        P1 = (0 +(radians(float(lineParts[1]))))-1.05
                        Joint_State = JointStateDY()
                        Joint_State.name = "right_lift_joint"
                        Joint_State.current_pos = P1
                        
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P6_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(p1)

                except:
                        rospy.logwarn("Unexpected error:left_lift_joint" + str(sys.exc_info()[0]))



        def _Broadcast_Right_Rotate_Joint(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 2):
                        pass
                try:
                        P1 = 0 - (radians(float(lineParts[1])))+ 1.57
                        Joint_State = JointStateDY()
                        Joint_State.name = "right_rotate_joint"
                        Joint_State.current_pos = P1
                        
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P5_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:right_rotate_joint" + str(sys.exc_info()[0]))

        def _Broadcast_left_gripper_Joint(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 2):
                        pass
                try:
                        P1 = 0 - (radians(float(lineParts[1])))+ 1.57
                        Joint_State = JointStateDY()
                        Joint_State.name = "left_gripper_joint"
                        Joint_State.current_pos = P1
                        
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P7_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:left_gripper_joint" + str(sys.exc_info()[0]))

        def _Broadcast_Right_gripper_Joint(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 2):
                        pass
                try:
                        P1 = 0 - (radians(float(lineParts[1])))+ 1.57
                        Joint_State = JointStateDY()
                        Joint_State.name = "right_gripper_joint"
                        Joint_State.current_pos = P1
                        
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P8_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:right_gripper_joint" + str(sys.exc_info()[0]))



        

        def _WriteSerial(self, message):
                self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
                self._SerialDataGateway.Write(message)

        def __init__(self,):
                '''
                Initializes the receiver class.
                port: The serial port to listen to.
                baudrate: Baud rate for the serial communication
                '''
                self.rate = rospy.get_param("~rate", 100.0)
                self.fake = rospy.get_param("~sim", False)
                self._Counter = 0
                rospy.init_node('gizmos')
                port = rospy.get_param("~port", "/dev/ttyACM0")
                baudRate = int(rospy.get_param("~baudRate", 115200))

                rospy.logwarn("Starting arm controller with serial port: " + port + ", baud rate: " + str(baudRate))

                # subscriptions
                rospy.Subscriber('right_lift_joint/command',Float64, self._HandleJoint_2_Command)
                rospy.Subscriber('right_rotate_joint/command',Float64, self._HandleJoint_3_Command)
                rospy.Subscriber('left_lift_joint/command',Float64, self._HandleJoint_6_Command)
                rospy.Subscriber('left_rotate_joint/command',Float64, self._HandleJoint_7_Command)
                rospy.Subscriber('left_gripper_joint/command',Float64, self._HandleJoint_8_Command)
                rospy.Subscriber('right_gripper_joint/command',Float64, self._HandleJoint_9_Command)
                self._SerialPublisher = rospy.Publisher('arm_serial', String, queue_size=5)
                self._P2_JointPublisher = rospy.Publisher('left_lift_joint/state', JointStateDY, queue_size=5)
                self._P3_JointPublisher = rospy.Publisher("left_rotate_joint/state", JointStateDY, queue_size=5)       
                self._P5_JointPublisher = rospy.Publisher("right_rotate_joint/state", JointStateDY, queue_size=5)
                self._P6_JointPublisher = rospy.Publisher("right_lift_joint/state", JointStateDY, queue_size=5)
                self._P7_JointPublisher = rospy.Publisher("left_gripper_finger_joint/state", JointStateDY, queue_size=5)
                self._P8_JointPublisher = rospy.Publisher("right_gripper_finger_joint/state", JointStateDY, queue_size=5)
                self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

        def Start(self):
                rospy.loginfo("Starting start function")
                self._SerialDataGateway.Start()
                message = 's \r'
                self._WriteSerial(message)

        def Stop(self):
                rospy.loginfo("Stopping")
                message = 'r \r'
                self._WriteSerial(message)
                sleep(5)
                self._SerialDataGateway.Stop()



        def _HandleJoint_2_Command(self, Command):
                """ Handle movement requests.
                             right_lift_joint
                send message in degrees 0 -180

                """
                v = Command.data      # angel request in radians
                self.right_lift = v
                v1 = (int(degrees(v))+ 90)
                message = 'b %d \r' % (v1)
                rospy.logwarn("Sending right_lift_joint command: " + (message))
                self._WriteSerial(message)

        def _HandleJoint_3_Command(self, Command):
                """ Handle movement requests.
                             right_rotate_joint
                send message in degrees 0 -180

                """
                v = Command.data      # angel request in radians
                self.right_rotate = v
                v1 =int(degrees(v))+ 90
                message = 'a %d \r' % (v1)
                rospy.logwarn("Sending right_rotate_joint command: " + (message))
                self._WriteSerial(message)


        def _HandleJoint_6_Command(self, Command):
                """ Handle movement requests.
                            left_lift_joint
                send message in degrees 0 -180

                """
                v = Command.data      # angel request in radians
                self.left_rotate = v
                v1 =int(degrees(v)) + 90
                message = 'd %d \r' % (v1)
                rospy.logwarn("Sending left_lift_joint command : " + (message))
                self._WriteSerial(message)

        def _HandleJoint_7_Command(self, Command):
                """ Handle movement requests.
                            left_rotatejoint
                send message in degrees 0 -180

                """
                v = Command.data      # angel request in radians
                self.right_elbow = v
                v1 =int(degrees(v))+ 90
                message = 'c %d \r' % (v1)
                rospy.logwarn("Sending left_rotate_joint command: " + (message))
                self._WriteSerial(message)

        def _HandleJoint_8_Command(self, Command):
                """ Handle movement requests.
                            left_gripper_joint
                send message in degrees 0 -180

                """
                v = Command.data      # angel request in radians
                self.left_gripper = v
                v1 =int(degrees(v))+ 90
                message = 'e %d \r' % (v1)
                rospy.logwarn("Sending left_gripper_joint command: " + (message))
                self._WriteSerial(message)

        def _HandleJoint_9_Command(self, Command):
                """ Handle movement requests.
                            right gripper_joint
                send message in degrees 0 -180

                """
                v = Command.data      # angel request in radians
                self.right_gripper = v
                v1 =int(degrees(v))+ 90
                message = 'f %d \r' % (v1)
                rospy.logwarn("Sending right_grippere_joint command: " + (message))
                self._WriteSerial(message)

if __name__ == '__main__':
        r_shoulder = Arm_Driver()
        try:
                r_shoulder.Start()
                rospy.spin()

        except rospy.ROSInterruptException:
                r_shoulder.Stop()

