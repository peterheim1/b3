#!/usr/bin/env python
'''

Created March, 2017

@author: Peter Heim

  sensor_driver.py - gateway to Arduino based arm controller
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
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64, Float32, Bool, Int16
from SerialDataGateway import SerialDataGateway
import sensor_msgs.msg as sensor_msgs
from phoenix_msgs.msg import Ir_State

class Sensor_Driver(object):
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
                        if (lineParts[0] == 's1'):
                                self._Sensor_state(lineParts)
                                return
                        


        def _Sensor_state(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 5):
                        pass
                try:
                    S1 = int(lineParts[1])
                    S2 = int(lineParts[2])
                    S3 = int(lineParts[3])
                    S4 = float(lineParts[4])
                    S5 = float(lineParts[5])
                    S6 = float(lineParts[6])
                    if S2 < 1:
                        left_rotate = float(0.02)
                    if S1 < 1:
                        right_rotate = -0.02
                    self.left_ir = S2
                    self.right_ir = S1
                    self.rear_bumper = S3
                    ir_state = Ir_State()
                    ir_state.right = int(S1)
                    ir_state.left = int(S2)
                    ir_state.rear = int(S3)
                    battery = sensor_msgs.BatteryState()
                    battery.header.stamp = rospy.Time.now()
                    battery.voltage = S4
                    battery.current = S6
                    battery.charge = float(S3)
                    battery.capacity = float('nan')
                    battery.design_capacity = float('nan')
                    battery.percentage = S4 / 0.126
                    battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
                    battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
                    battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
                    battery.present = True
                    battery.location = ""
                    battery.serial_number = ""
                    
                    #self._S1_Publisher.publish(left_rotate + right_rotate)# angle
                    #self._S2_Publisher.publish(S2)#right ir
                    self._S3_Publisher.publish(S3)#rear bumper
                    self._S4_Publisher.publish(battery)#voltage
                    self._S5_Publisher.publish(S5)#drive voltage
                    self._S6_Publisher.publish(S6)#current
                    self._S7_Publisher.publish(ir_state)#ir_state
                    #rospy.logwarn(S1)

                except:

                    rospy.logwarn("Unexpected error:sensor state" + str(sys.exc_info()[0]))




        

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
                self.left_rotate = 0
                self.right_rotate = 0
                self.rear_bumper = 1
                rospy.init_node('gizmos_sensors')
                port = rospy.get_param("~port", "/dev/ttyACM0")
                baudRate = int(rospy.get_param("~baudRate", 115200))

                rospy.logwarn("Starting sensor controller with serial port: " + port + ", baud rate: " + str(baudRate))

                
                self._VelocityCommandPublisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
                self._SerialPublisher = rospy.Publisher('sensor_state', String, queue_size=5)
                #self._S1_Publisher = rospy.Publisher('angle', Float64, queue_size=5)
                #self._S2_Publisher = rospy.Publisher('left_ir', Bool, queue_size=5)
                self._S3_Publisher = rospy.Publisher('rear_bumper', Int16, queue_size=5)
                self._S4_Publisher = rospy.Publisher('system_battery', sensor_msgs.BatteryState, queue_size=5)
                self._S5_Publisher = rospy.Publisher('drive_battery', Float64, queue_size=5)
                self._S6_Publisher = rospy.Publisher('system_current', Float64, queue_size=5)
                self._S7_Publisher = rospy.Publisher('ir_state', Ir_State, queue_size=5)
                
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





if __name__ == '__main__':
        sensor_state = Sensor_Driver()
        try:
                sensor_state.Start()
                rospy.spin()

        except rospy.ROSInterruptException:
                sensor_state.Stop()

