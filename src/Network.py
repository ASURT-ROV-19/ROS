#!/usr/bin/env python3

import rospy
import sys
import selectors
import socket
from subprocess import PIPE, Popen
# from VideoStream import *
import time
from orca.msg import Dict ,dictionary
from std_msgs.msg import Int8 , Empty
class TCP :
    def __init__(self,selector,ip,port, streamingIP):
        self._buffer_size = 4096
        self.Num_Of_tokens = 5
        self.sample_time=1

        # ============ ROS ======================
        self.Node = "Network"
        self.my_dict = dictionary()
        rospy.init_node(self.Node)
        self.Publisher_TCP =                  rospy.Publisher("TCP",dictionary,queue_size=10)
        self.Publisher_TCP_Error =            rospy.Publisher("TCP_ERROR",Empty,queue_size=1)
        self.Publisher_Pilot_Enable =         rospy.Publisher("Pilot_Enable",Int8,queue_size=1)
        self.Publisher_Control   =            rospy.Publisher("Control",Empty,queue_size=10)

        # =======================================

        # For PI 19
        self.RaspberryPi_IP = '10.1.1.15'
        self.Laptop_IP = '10.1.1.14'
        # For Local
        self.RaspberryPi_IP = '127.0.0.1'
        self.Laptop_IP = '127.0.0.1' # sink ( Laptop's address )
        self.Port = 9005
        self.stream_Ports = ['5022','5000','10000']

        self._ip = self.RaspberryPi_IP
        self._port = self.Port
        self._socket = None
        self._conn = None
        self._client_address = None
        self._stream_disconect = False
        self.pilot_enable = False
        self._selector = selectors.DefaultSelector()

        self._create_Socket()
        self._bind_Listen()
        # self.Turn_On_Cameras()
        print(self._selector)

        self.main_Loop()
    def Convert_Dict_to_structure(self,dict_joystick:dict):
        self.my_dict.dictionary.clear()
        for key in dict_joystick:
            temp_dict = Dict()
            temp_dict.key = key
            temp_dict.value= dict_joystick[key]
            self.my_dict.dictionary.append(temp_dict)

    def Turn_On_Cameras(self):

        self.pipeline1 = "v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! rtpjpegpay ! udpsink host=" + self.Laptop_IP + " port=" + self.stream_Ports[0]
        self.pipeline2 = "v4l2src device=/dev/video2 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! rtpjpegpay ! udpsink host=" + self.Laptop_IP + " port=" + self.stream_Ports[2]
        self.pipeline3 = "v4l2src device=/dev/video2 ! video/x-raw,width=640,height=480 ! jpegenc ! rtpjpegpay ! udpsink host=10.1.1.14 port=1234"
        # for Laptop's Camera
        self.pipeline1 = "v4l2src ! video/x-raw,width=640,height=480 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5022 sync=false"
#        self.pipeline2 = "v4l2src ! video/x-raw,width=640,height=480 ! jpegenc ! rtpjpegpay ! multiudpsink clients=127.0.0.1:1234,127.0.0.1:5022"

        # self.Camera = Gstreamer(self.pipeline1)
        # self.Camera2 = Gstreamer(self.pipeline2)
        # self.cam= Gstreamer(self.pipeline3) # endo scope

#        self.Camera.start()
 #       self.Camera2.start()
  #      self.cam.start()    # endo scope

    def _create_Socket(self):
        self._socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)

    def get_socket(self):
        return self._socket
    def get_conn(self):
        return self._conn
    def _bind_Listen(self):
        try:
            self._socket.bind( (self._ip,self._port) )
            self._socket.listen(1)
        except socket.error:
            print('Close The Code from htop->F9->9')
            print('Check IP and Port')
            quit()
            return

        self._socket.setblocking(False)
        self._selector.register(self._socket,selectors.EVENT_READ,self._acceept)

    def _acceept(self):
        # if someone tries to connect the pi while it is already connected
        if self._conn is not None:
            # ========== Close The old and open new Connection ===============================
            # self.close()
            # self._conn , self._client_address = self._socket.accept()
            # self._selector.register(self._conn,selectors.EVENT_READ,self._recv)
            # print("D5ool Rayaaaaaaaaaaaaaaaaaa2")

            # =========== Pull the Event from Selectors and close it and keep the old =========
            print("someone tries to connect pi")
            temp_conn , _ = self._socket.accept()
            temp_conn.close()
            return
        # ===================== TCP Server ========================
        self._conn , self._client_address = self._socket.accept()
        print('Connected ya ray2      ',self._client_address)

        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)

        self._conn.setblocking(False)
        self._selector.register(self._conn,selectors.EVENT_READ,self._recv)
        # =========================================================

    def send_Temp(self,event,sensor_temp):
        # get pi temp
#        print(event)
        process = Popen(['vcgencmd', 'measure_temp'], stdout=PIPE)
        output, _error = process.communicate()
        pi_temp = output.decode()
        pi_temp = "\nPI_"+pi_temp
        print(pi_temp)

        temp = str(sensor_temp)
#        print("Temp:",temp)
        self._conn.sendall(temp.encode())
        print("Temp:",temp)

    def Check_for_Special_Msgs(self,msg):
        if msg == "PID=1":
            self.pilot_enable = 1
            self.Publisher_Pilot_Enable.publish(self.pilot_enable)
            rospy.loginfo("PID ===  1 ")
            return True

        elif msg == "PID=0":
            self.pilot_enable = 0
            self.Publisher_Pilot_Enable.publish(self.pilot_enable)
            rospy.loginfo("PID ===== 0")
            return True

        return False
    def _recv(self):
        data = self._conn.recv(self._buffer_size).decode(encoding="UTF-8")
#        print(data)
        if not data: # disconnection return empty string ""
            self.close()
            return

        Special = self.Check_for_Special_Msgs(data)
        if Special:
            return

        try:
            Qtstrings=self.Split_to_Dict(data)
        except:
            print("Msg 8areeebaa Moreebaaaa 3ageeebaa")
            print("Exception Split ray2:",sys.exc_info()[0])

            return
        if Qtstrings == None:
            return


        for string in Qtstrings :
            # self.Convert_Dict_to_structure(string)
            # self.Publisher_TCP.publish(self.my_dict)
            self.Publisher_TCP.publish(string)

    def close(self):
        self.Publisher_TCP_Error.publish()
        self.pilot_enable = False

        if self._conn is not None:
            self._selector.unregister(self._conn)
            self._conn.close()
            self._conn = None

    def Split_to_Dict(self,qt_string: str):
        Qt_strings = []

        msgs = qt_string.split('&')
        n = len (msgs)
        del msgs[ n -1 ]

        if n > 1 :
            for msg in msgs :
                temp_msg = msg.split(',')

                if len(temp_msg) != self.Num_Of_tokens:
                    print("Num of Terms(",len(temp_msg),") Dosn't Equal",self.Num_Of_tokens)
                    return None                

                temp_Qt_string_dict = []
                for term in temp_msg:
                    temp = Dict()
                    temp_list = term.split("=")
                    temp.key = temp_list[0] 
                    temp.value = int(temp_list[1])

                    temp_Qt_string_dict.append(temp)
                Qt_strings.append(temp_Qt_string_dict)
            # print(Qt_strings)
            return Qt_strings

        elif n <= 1:
            print('num of tokens error')
            return None

        return Qt_strings

    def hard_Shutdown_Recreate_Socket(self):
        self.close()
        self._selector.unregister(self._socket)
        self._socket.shutdown(2)
        self._socket.close()
        self._socket = None
        self._create_Socket()
        self._bind_Listen()
        print(" .................New Socket Created ...............")
  
    def main_Loop(self):
        print('Wait for zeft Qt')
        while not rospy.is_shutdown():
            try:
                events = self._selector.select(timeout=self.sample_time)
                for key, mask in events:
                    key.data()

#                self.Publisher_Control.publish()

            except (TimeoutError,ConnectionResetError) :
                self.hard_Shutdown_Recreate_Socket()
            except KeyboardInterrupt:
                print(' Tari2 El Salama Enta')
                self.close()
                self._selector.close()
                self.Camera.close()
                self.Camera2.close()
                return

            # Important
            except:
                print("Exception Msh Ray2:",sys.exc_info()[0])
                self.hard_Shutdown_Recreate_Socket()

tcp = TCP(1,1,1,1)
