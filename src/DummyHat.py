#!/usr/bin/env python3
import time ,rospy
from orca.msg import Dict, dictionary
from std_msgs.msg import Int8 , Int32 , Empty , Float64

class Hat:
    def __init__(self):
#        self._hat = Adafruit_PCA9685.PCA9685()
#        self._hat.set_pwm_freq(frequency)
        rospy.init_node("Hat_Node")
        self._hat= "Ray2"

        self.address = 0x40
        self.frequency = 50
        self.delay = 0.000020 # us
        self._devices = {}
        self.Zero_Vertical=305
        self.channelZ1 = None
        self.channelZ2 = None
        self.Magazine_Channel = None

        self.Max_Magazine = 1
        self.Zero_Magazine= 0
        self.Min_Magazine = -1

        self.Enable = False
        self.pilot_enable = False

        self.channel_micro = 3
        self.zero_micro = 0
        self.forward_micro = 4000

        # for Magazine
        self.IN1 = 20 
        self.IN2 = 21
        self.ENA = 16
        self.ENA_PWM = 0

        # for Pulley 
        self.IN3 = 26
        self.IN4 = 19
        self.ENB = 13
        self.ENB_PWM = 0

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.IN1, GPIO.OUT)
        # GPIO.setup(self.IN2, GPIO.OUT)
        # GPIO.setup(self.ENA, GPIO.OUT)

        # GPIO.setup(self.IN3, GPIO.OUT)
        # GPIO.setup(self.IN4, GPIO.OUT)
        # GPIO.setup(self.ENB, GPIO.OUT)

        self.micro_gpio = 27
        # GPIO.setup(self.micro_gpio, GPIO.OUT)


        # Set the Speed of All Motors
    def add_Device(self,name,channel,zero_value):
        self._devices[name] = {'channel':channel , 'zero':zero_value , 'current': zero_value}
        # self._hat.set_pwm(channel,0,int(zero_value))

        if name == "Vertical_Right":
            self.channelZ1 = channel
            print ("Channel Z =",self.channelZ1)
        elif name == "Vertical_Left":
            self.channelZ2 = channel
            print ("Channel Z =",self.channelZ2)
        elif name == "Magazine_Servo":
            self.Magazine_Channel = channel
            print ("Magazine Channel :",self.Magazine_Channel)

    def Raspberry_pi_Power(self,channel,value):
        # self._hat.set_pwm(channel,0,value)
        pass

    def Magazine_Servo(self):
        # take action when pulley is stoped
        if self.ENB_PWM == 0:
            pwm = self._devices['Magazine_Servo']['current']
            if pwm == self.Max_Magazine:
                # GPIO.output(self.ENA, 1)
                # GPIO.output(self.IN1, 1)
                # GPIO.output(self.IN2, 0)
                self.ENA_PWM = 1

            elif pwm == self.Zero_Magazine:
                # GPIO.output(self.ENA, 0)
                # GPIO.output(self.IN1, 0)
                # GPIO.output(self.IN2, 0)
                self.ENA_PWM = 0

            elif pwm == self.Min_Magazine:
                # GPIO.output(self.ENA, 1)
                # GPIO.output(self.IN1, 0)
                # GPIO.output(self.IN2, 1)
                self.ENA_PWM = 1

        else :
            self._devices['Magazine_Servo']['current'] = self.Zero_Magazine
            # GPIO.output(self.ENA, 0)
            # GPIO.output(self.IN1, 0)
            # GPIO.output(self.IN2, 0)
            self.ENA_PWM = 0
            print("Stop Magazine from Magazine El72 el DC Chopper ya Sa3eeeed")
        print("ENA_PWM",self.ENA_PWM)


    def _updatePWM(self,pwms_structure:dictionary):
        pwms = self.Convert_structure_to_Dict(pwms_structure)
        for device_name in pwms:
            # Check for PID Mode Controlled By Pilot ==================================
            if self.pilot_enable:
                # Check for PID Enable Controlled by Equation
                if ( device_name == "Vertical_Right" or device_name == "Vertical_Left" ) and self.Enable == True:
                    continue
             # ========================================================================
            # Check for Repetetion
            if self._devices[device_name]['current'] == pwms[device_name]:
                continue

            self._devices[device_name]['current'] =pwms[device_name]

            if device_name == 'Magazine_Servo':
                self.Magazine_Servo()
                continue
                        
            # self._hat.set_pwm(self._devices[device_name]['channel'],0,int(self._devices[device_name]['current']))
            time.sleep(self.delay)
        print(pwms)

    def Pilot_Enable(self,enable):
        self.pilot_enable = enable.data
        if not self.pilot_enable :
            # self._hat.set_pwm(self.channelZ1,0,self.Zero_Vertical)
            # self._hat.set_pwm(self.channelZ2,0,self.Zero_Vertical)
            pass

    def Enable_PID(self,value):
        self.Enable = value.data

    def Micro_ROV(self,pwm):
            # GPIO.output(self.micro_gpio,pwm)
            print("Micro GPIO ",pwm)

    def Pulley(self,pwm):
        if self._devices['Magazine_Servo']['current'] != self.Zero_Magazine or self.ENA_PWM == 1:           

            self._devices['Magazine_Servo']['current'] = self.Zero_Magazine
            # GPIO.output(self.ENA, 0)
            # GPIO.output(self.IN1, 0)
            # GPIO.output(self.IN2, 0)
            self.ENA_PWM = 0
            print("Stop Magazine from Pulley El72 el DC Chopper ya Sa3eeeed")

        if pwm == 0:
            # GPIO.output(self.ENB, 0)
            # GPIO.output(self.IN3, 0)
            # GPIO.output(self.IN4, 0)
            self.ENB_PWM = 0            

        elif pwm == 1:
            # GPIO.output(self.ENB, 1)
            # GPIO.output(self.IN3, 1)
            # GPIO.output(self.IN4, 0)
            self.ENB_PWM = 1      

        elif pwm == -1:
            # GPIO.output(self.ENB, 1)
            # GPIO.output(self.IN3, 0)
            # GPIO.output(self.IN4, 1)
            self.ENB_PWM = 1      
        
        print("ENB PWM",self.ENB_PWM)

    def Clean_GPIO(self):
            
            # GPIO.output(self.ENA, 0)
            # GPIO.output(self.ENB, 0)

            # GPIO.output(self.IN1, 0)
            # GPIO.output(self.IN2, 1)
            # GPIO.output(self.IN3, 0)
            # GPIO.output(self.IN4, 1)

            # GPIO.output(self.micro_gpio,0)
            print("HARD CLEAN GPIO")


    def PID_Control(self, pwm_z):
        pwm = pwm_z.data
        if self.Enable:
            if abs ( self._devices['Vertical_Left']['current'] - pwm ) <= 2:
                return

            # self._hat.set_pwm(self.channelZ1,0,int(pwm))
            time.sleep(self.delay)
            # self._hat.set_pwm(self.channelZ2,0,int(pwm))
            time.sleep(self.delay)

            self._devices['Vertical_Left']['current'] =pwm
            self._devices['Vertical_Right']['current'] =pwm
            print("control effort value::: ",pwm)

    def Convert_structure_to_Dict(self,dic:dictionary):
        new_dict = {}
        for i in dic.dictionary:
            new_dict[i.key] =  i.value
        return new_dict

    def ROS_Spin_Loop(self):
        rospy.Subscriber("HAT",dictionary,self._updatePWM)
        rospy.Subscriber("control_effort",Float64,self.PID_Control)
        rospy.Subscriber("ENABLE_PID",Int8,self.Enable_PID)
        rospy.Subscriber("Pilot_Enable",Int8,self.Pilot_Enable)

        rospy.spin()

Zero_thruster = 305
Zero_Servo = 225
Zero_Magazie = 0 

hat = Hat()
hat.add_Device('Left_Front', 5, Zero_thruster)
hat.add_Device('Right_Front', 2, Zero_thruster)
hat.add_Device('Right_Back', 13,Zero_thruster)
hat.add_Device('Left_Back',15 , Zero_thruster)
hat.add_Device('Vertical_Right', 9, Zero_thruster)
hat.add_Device('Vertical_Left', 11, Zero_thruster)
hat.add_Device('Main_Cam',0,Zero_Servo)
hat.add_Device('Back_Cam',1,Zero_Servo)
hat.add_Device('Magazine_Servo',3,Zero_Magazie)
hat.Raspberry_pi_Power(7,305)


hat.ROS_Spin_Loop()
