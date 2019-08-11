#!/usr/bin/env python3

# Remove Setpoint Publisher
import time
import math
import rospy
from orca.msg import Dict , dictionary
from std_msgs.msg import Int8 , UInt32 , Empty ,Float64 , Int32

class Motion:
    def __init__(self,Qt_String):

        self.Enable_GPIO = False

        self.emit_signal = None
        self._Qt_String = Qt_String.copy()
        # =========== Constants ===========
        self.Zero_thruster = 305
        self.Zero_Servo = 225
        self.Servo_min = 100
        self.Servo_max = 360
        self.Servo_max = 390
        self.Brake = 240
        self.Forward = 400
        self.Joystick_min = -100
        self.Joystick_max = 100
        self.Rotation_Efficiency = 0.20
        self.Rotation_Speed = 0.40
        self.PWM_Map_Coff = (1 / self.Joystick_max) * (self.Forward - self.Zero_thruster)
        self.PWM_Map_Coff_reverse = (1 / self.Joystick_max) * (self.Zero_thruster-self.Brake)

        self.Zero_Magazie = 0
        self.Max_Magazine = 1
        self.Min_Magazine = -1

        self.CoffZ_reverse = 0.7
        self.CoffZ = 1
        self.camera_step = 5
        self.delay = False
        self.pid_flag = False

        self.Switch_pin = 26
        self.Magazine_flag = False
        if self.Enable_GPIO:
         self.Setup_GPIO()

        # ============= ROS ==================
        rospy.init_node("Motion")
        self.my_dict = dictionary()
        self.Publisher_Hat          =  rospy.Publisher("HAT",dictionary,queue_size=1)
        self.Publisher_ENABLE_PID   =  rospy.Publisher("ENABLE_PID",Int8,queue_size=1)
        self.Publisher_Pilot_Enable =  rospy.Publisher("state",Float64,queue_size=1) 
        self.Publisher_SetPoint     =  rospy.Publisher("setpoint",Float64,queue_size=1) 

        # =========== Motors==============
        self._horizontalMotors= []
        self._verticalMotors  = []
        self._servos =  []
        self._lights =  []
        self._horizontalMotors.append(Dict("Left_Front",self.Zero_thruster ) )
        self._horizontalMotors.append(Dict("Right_Front",self.Zero_thruster ) )
        self._horizontalMotors.append(Dict("Right_Back",self.Zero_thruster ) )
        self._horizontalMotors.append(Dict("Left_Back",self.Zero_thruster ) )
        self._verticalMotors.append(Dict("Vertical_Right",self.Zero_thruster ) )
        self._verticalMotors.append(Dict("Vertical_Left",self.Zero_thruster ) )
        self._servos.append(Dict("Main_Cam",self.Zero_Servo ) )
        self._servos.append(Dict("Back_Cam",self.Zero_Servo ) )
        self._servos.append(Dict("Magazine_Servo",self.Zero_Magazie ) )
        # ======= initialization ===========
        self._stopHorizontalMotors()
        self._stopVerticalMotors()
        self._setCamToNormalPosition()
        # ==================================

        self.ROS_Spin_Loop()

    def Convert_structure_to_Dict(self,dic:dictionary):
        new_dict = {}
        for i in dic.dictionary:
            new_dict[i.key] =  i.value
        return new_dict

    def Convert_Dict_to_structure(self,dict_joystick:dict):
        self.my_dict.dictionary.clear()
        for key in dict_joystick:
            temp_dict = Dict()
            temp_dict.key = key
            temp_dict.value= dict_joystick[key]
            self.my_dict.dictionary.append(temp_dict)

    def Setup_GPIO(self):
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(pin, GPIO.OUT)
        pass
    def Switch_on_off_Magazine(self, enable):
        if enable and self.Magazine_flag:
            if self.Enable_GPIO:
             self.Setup_GPIO()

            self.Magazine_flag = False
            time.sleep(0.05)

        elif not enable and not self.Magazine_flag:
            if self.Enable_GPIO:
             self.Setup_GPIO()
#            GPIO.cleanup
#            print("Switch OFF")
            self.Magazine_flag = True
            time.sleep(0.05)

    def _stopHorizontalMotors(self):
        self.set_value_Horizontal("Left_Front",self.Zero_thruster ) 
        self.set_value_Horizontal("Right_Front",self.Zero_thruster )
        self.set_value_Horizontal("Right_Back",self.Zero_thruster  )
        self.set_value_Horizontal("Left_Back",self.Zero_thruster  )

    def _stopVerticalMotors(self):
        self.set_value_Vertical("Vertical_Right",self.Zero_thruster ) 
        self.set_value_Vertical("Vertical_Left",self.Zero_thruster ) 

    def _setCamToNormalPosition(self):
        self.set_value_Servoes("Main_Cam",self.Zero_Servo )
        self.set_value_Servoes("Back_Cam",self.Zero_Servo ) 
        self.set_value_Servoes("Magazine_Servo",self.Zero_Magazie ) 

    def Map (self,x,motor):
        if x <= 0 and motor == "Left_Front" :
            return self.Zero_thruster + x * self.PWM_Map_Coff_reverse
        elif x <= 0 and motor == "Right_Front" :
            return self.Zero_thruster + x * self.PWM_Map_Coff_reverse
        elif x <= 0 and motor == "Right_Back" :
            return self.Zero_thruster + x * self.PWM_Map_Coff_reverse
        elif x <= 0 and motor == "Left_Back" :
            return self.Zero_thruster + x * self.PWM_Map_Coff_reverse

        elif motor == "Left_Front" :
            return self.Zero_thruster + x * self.PWM_Map_Coff
        elif motor == "Right_Front" :
            return self.Zero_thruster + x * self.PWM_Map_Coff * 0.8
        elif motor == "Right_Back" :
            return self.Zero_thruster + x * self.PWM_Map_Coff * 0.8
        elif motor == "Left_Back" :
            return self.Zero_thruster + x * self.PWM_Map_Coff
    @staticmethod
    def map(value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

    def get_value_QtString(self,key:str):
        for dicct in self._Qt_String:
            if dicct.key == key :
                return dicct.value

    def set_value_Horizontal(self,key:str,value:Int32):
        for i in self._horizontalMotors:
            if key == i.key :
                i.value = value
                return
    def set_value_Vertical(self,key:str,value:Int32):
        for i in self._verticalMotors:
            if key == i.key :
                i.value = value
                return
    def set_value_Servoes(self,key:str,value:Int32):
        for i in self._servos:
            if key == i.key :
                i.value = value
                return        

    def calculateHorizontalMotors_19(self):
        x =self.get_value_QtString('x')
        y =self.get_value_QtString('y')
        r =self.get_value_QtString('r')

        theta = math.atan2(y, x)
        # ============== Skip ============================
        # Check https://ibb.co/ir10AV
        # R = math.hypot(x, y) * (max(abs(math.sin(theta)), abs(math.cos(theta))))
        # ================================================

        # Convert to Circle Coordinates to limit the maximum speed
        # Check https://www.xarg.org/2017/07/how-to-map-a-square-to-a-circle/
        # ================================================
        # The Circle Coordination Equation support [-1,1] range
        X = x / self.Joystick_max
        Y = y / self.Joystick_max

        Xc_2 = X * X * (1 - Y * Y / 2)
        Yc_2 = Y * Y * (1 - X * X / 2)

        R = math.sqrt(Xc_2 + Yc_2) * self.Joystick_max
        # ================================================
        # Axis Rotation
        theta -= math.pi / 4
        # Coefficient to Get the Best Speed of Motors
        # Check https://ibb.co/fYUZ4q   # Check https://ibb.co/dSFqPf
        coff = max(abs(math.sin(theta)), abs(math.cos(theta)))
        # ============ ( C ) is Rotation Efficiency =================
        C = self.map(abs(r),0,self.Joystick_max,0,self.Rotation_Efficiency)
        # C = self.Rotation_Efficien
        if R == 0 :
            C = self.Rotation_Speed
        # ===========================================================

        R_Cos_Coff = R * math.cos(theta) / coff
        R_Sin_Coff = R * math.sin(theta) / coff

        Motor1 = (1-C) * R_Cos_Coff + C *r
        Motor2 = (1-C) * R_Sin_Coff - C *r
        Motor3 = (1-C) * R_Cos_Coff - C *r
        Motor4 = (1-C) * R_Sin_Coff + C *r


        # Map the Joystick Coordinates to PWM Coordinates

        self.set_value_Horizontal ("Left_Front" , int(self.Map(Motor1,'Left_Front' ) ) )
        self.set_value_Horizontal ("Right_Front", int(self.Map(Motor2,'Right_Front') ) )
        self.set_value_Horizontal ("Right_Back" , int(self.Map(Motor3,'Right_Back' ) ) )
        self.set_value_Horizontal ("Left_Back"  , int(self.Map(Motor4,'Left_Back'  ) ) )

        # print(pwm)
    def calculateVerticalMotors_19(self):
        # steps = real pwm range / freq  = 250 M / 50 = 5 M (5000000)
        # z = Motion.map (self._Qt_String['z'] , self.Joystick_min,self.Joystick_max,300,500)
        Z = self.get_value_QtString('z')

        if Z >= 0:
            z = self.Zero_thruster + Z * self.CoffZ
        elif Z < 0 :
            z = self.Zero_thruster + Z * self.CoffZ_reverse

        self.set_value_Vertical('Vertical_Right' ,int(z) )
        self.set_value_Vertical('Vertical_Left'  ,int(z) )

        if Z == 0:
            self.Publisher_ENABLE_PID.publish(1)
            if not self.pid_flag:
                self.Publisher_SetPoint.publish(1)
                self.pid_flag= True
                print("Enable True")

        else:
            self.Publisher_ENABLE_PID.publish(0)

            if self.pid_flag :
                print("Enable False")
                self.pid_flag = False

    def moveCamera(self):
        pass
        # if self._Qt_String['cam'] == 1 and self._servos['Main_Cam'] > self.Servo_min  :
        #     self._servos['Main_Cam'] -= self.camera_step
        #     if self._servos['Back_Cam'] > self.Servo_min:
        #       self._servos['Back_Cam'] -= self.camera_step
        # elif self._Qt_String['cam'] == 4 and self._servos['Main_Cam'] < self.Servo_max:
        #     self._servos['Main_Cam'] += self.camera_step
        #     if self._servos['Back_Cam'] <= 305:
        #       self._servos['Back_Cam'] += self.camera_step
        # elif self._Qt_String['cam'] == 2:
        #     self.Switch_on_off_Magazine(True)
        #     self._servos['Magazine_Servo'] = self.Min_Magazine
        # elif self._Qt_String['cam'] == 8:
        #     self.Switch_on_off_Magazine(True)
        #     self._servos['Magazine_Servo'] = self.Max_Magazine
    def print_PWM(self):
        print(self._horizontalMotors,self._verticalMotors)

    def ROS_Spin_Loop(self):
        rospy.Subscriber("TCP",dictionary,self.TCP)
        rospy.Subscriber("TCP_ERROR",Empty,self.TCP_ERROR)
        rospy.spin()

    def TCP_ERROR(self,empty):

        self._stopVerticalMotors()
        self._stopHorizontalMotors()
        self._setCamToNormalPosition()
        self.Publisher_ENABLE_PID.publish(0)
        self.Publisher_Pilot_Enable.publish(0)
    
        self.Hat_PWM()
        rospy.loginfo("TCP ERROR  8adaro beena")
            
    def TCP(self,data_dict):
        self._Qt_String = data_dict.dictionary
        try:
         self.calculateVerticalMotors_19()
         self.calculateHorizontalMotors_19()
        except KeyError:
         print("Key Error Ray2")
         return
        if self.get_value_QtString('cam') != 0:
             self.moveCamera()
             self.delay = True
        else :
             self.set_value_Servoes('Magazine_Servo', self.Zero_Magazie)
             self.Switch_on_off_Magazine(False)

        self.Hat_PWM()

    def Hat_PWM(self):

        pwm = self._horizontalMotors + self. _verticalMotors + self._servos
        self.Publisher_Hat.publish(pwm)

Qt_String = [Dict('x',0) ,Dict('y',0),Dict('r',0),Dict('z',0),Dict('cam',0) ]
print(Qt_String)
motion = Motion(Qt_String)
