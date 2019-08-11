#!/usr/bin/env python3
import time
import rospy
from orca.msg import Dict, dictionary
from std_msgs.msg import Int8 , Int32 , Empty

class MS5837_30BA():
    def __init__(self):
        self.y= 0.0
    def setFluidDensity(self,x):
        self.x = x
    def depth(self):
        self.y = 100
        return self.y
    def temperature(self,x):
        return 37

    def init(self):
        return True
    def read(self):
        return True

class PID:
    """PID Controller
    """

    def __init__(self):
        rospy.init_node("PID_Node")

        self.Publisher_PID = rospy.Publisher("PID",Int32 , queue_size=10)
        self.pilot_enable = False
        self.enable = False

        self.Kp = 250
        self.Ki = 53
        self.Kd = 35

        self.sensor = MS5837_30BA()
        self.sensor.setFluidDensity(1000)  # kg/m^3

        self.sample_time = 0.01
        self.current_time = time.time()
        self.last_time = self.current_time

        self.pwm_zero = 305
        self.out_max = 400
        self.out_min = 240
        self.zero_offset = 305
        self.fwd_zero_offset = 317
        self.bwd_zero_offset = 296

        try:
          if not self.sensor.init():
              exit(1)
          if not self.sensor.read():
              exit(1)
        except:
            pass

        self.depth = 0.0
        self.sensor_offset = self.sensor.depth()
        self.SetPoint = 1
        self.clear()

    def clear(self):
#        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0
    def SIGNAL_Referance(self,emit_signal):
        self.emit_Signal=emit_signal

    def update(self, set_point, feedback_value):
        self.SetPoint = set_point
        error = self.SetPoint - feedback_value
#        print("Set Point: "+str(self.SetPoint))
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
#            print("output"+str(self.output))
            # add pwm zero offset to output
            self.output += self.zero_offset

            # account for min and max ranges
            if self.output > self.out_max:
                self.output = self.out_max
            elif self.output < self.out_min:
                self.output = self.out_min

            # account for dead zone
            if (self.output > self.zero_offset) and (self.output < self.fwd_zero_offset):
                self.output = self.fwd_zero_offset
            elif (self.output < self.zero_offset) and (self.output > self.bwd_zero_offset):
                self.output = self.bwd_zero_offset

        self.output = int(self.output)

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

    def calibrate_sensor(self,zero_reading):
        self.sensor_offset =  0

    def get_Temp(self,event):
        # try:
        #     self.sensor.read()
        # except OSError:
        #     return

        # temp = self.sensor.temperature(ms5837.UNITS_Centigrade)
        # self.emit_Signal ('Send_Temp',temp )
        pass
    def set_Setpoint_to_depth(self,flag):
        if flag and self.pilot_enable:
            try:
                self.sensor.read()
                self.SetPoint = self.sensor.depth() - self.sensor_offset
                print("SetPoint:",self.SetPoint)
            except OSError:
                print("SetPoint OSError")
                self.Publisher_PID.publish(0)

    def Pilot_Enable(self,enable):
        self.pilot_enable = enable.data
        print("Pilot_Enable:", self.pilot_enable)
        self.set_Setpoint_to_depth(self.pilot_enable)

    def Enable_PID(self,enable):
        self.enable = enable.data

    def Control_PID(self,empty):
        try:
#
            if self.pilot_enable and self.enable:
               # ==============================================================
                try:
                    if self.sensor.read():
                        self.depth = self.sensor.depth()

                        self.depth = float(self.depth) - self.sensor_offset

                        self.update(self.SetPoint, self.depth)

                        print("Depth: %.3f m" % (self.depth),"pwm: " + str(self.output))
                        self.Publisher_PID.publish(self.output)
                    else :
                        print("Sensor read unavalable,\n")

                except OSError :
                    print ("OSERROR PID RAY2")
                except :
                    print("ERROR IN PID LOOP")

                # ==============================================================
        except KeyboardInterrupt:
            self.Publisher_PID.publish(self.pwm_zero)

    def ROS_Loop_Spin(self):
        rospy.Subscriber("Control",Empty,self.Control_PID)
        rospy.Subscriber("SetPoint",Int8,self.set_Setpoint_to_depth)
        rospy.Subscriber("ENABLE_PID",Int8,self.Enable_PID)
        rospy.Subscriber("Pilot_Enable",Int8,self.Pilot_Enable)

        rospy.spin()

pid = PID()
pid.ROS_Loop_Spin()
