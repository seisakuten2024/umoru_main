import rospy
import time
# import module
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from jsk_recognition_msgs.msg import HumanSkeletonArray
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import json
from PIL import Image
from io import BytesIO
import numpy as np
import MeCab
# from wordcloud import WordCloud, ImageColorGenerator
import matplotlib.pyplot as plt
# from openai import OpenAI
import re
import datetime
import requests
from sound_play.libsoundplay import SoundClient
from umoru_arm import MotionClient
from audio_common_msgs.msg import AudioData
import math
from collections import deque
from scipy.signal import lfilter


global CURRENT_UMORU_STATE, INTERACTING_FLAG, FACE_FIND_FLAG, TIME_CONTROLLER_LIST, LAST_TIME
CURRENT_UMORU_STATE = 0
INTERACTING_FLAG = False
FACE_FIND_FLAG = False
TIME_CONTROLLER_LIST = []
LAST_TIME = None


class startAndEndFlag():
    def __init__(self):
        print("init start and end flag")
        # self.sub = rospy.Subscriber("/camera/face_skeleton_with_depth/output/pose", HumanSkeletonArray, self.callback, queue_size=1)
        self.sub = rospy.Subscriber("/camera/color/face_pose_estimation/output/skeleton", HumanSkeletonArray, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        self.face_appeared_time = None
        
    def callback(self, data):
        global FACE_FIND_FLAG
        global INTERACTING_FLAG
        global TIME_CONTROLLER_LIST
        global LAST_TIME
        pub_msg_state = Int16()
        skeletons = data.skeletons
        if INTERACTING_FLAG == False and (LAST_TIME == None or rospy.get_time() - LAST_TIME >= 10):
            print("INTERACTING_FLAS is ", INTERACTING_FLAG)
            print("FACE FIND FLAG is ", FACE_FIND_FLAG)
            if len(skeletons) == 0:
                FACE_FIND_FLAG = False
            elif len(skeletons) != 0:
                # print(FACE_FIND_FLAG, self.face_appeared_time)
                if FACE_FIND_FLAG == True:
                    if rospy.get_time() - self.face_appeared_time >= 3:
                        INTERACTING_FLAG = True
                        print("================= found face for 3 seconds ======================")
                        TIME_CONTROLLER_LIST = []
                        TIME_CONTROLLER_LIST.append(rospy.get_time())
                        pub_msg_state.data = 1
                        self.publish(pub_msg_state)
                        FACE_FIND_FLAG = False
                else:
                    self.face_appeared_time = rospy.get_time()
                    FACE_FIND_FLAG = True
                    # print("face found", self.face_appeared_time)
    def publish(self, data):
        self.pub.publish(data)

# class pressureSub():
#     """
#     気圧センサによって加えるポイントを決定する
#     """
#     def __init__(self):
#         print("init pressure subscriber")
#         # Subscriberの作成
#         self.sub = rospy.Subscriber("/sensor", Int8, self.callback, queue_size=1)
#         self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
#         self.recent_pressure = deque([], maxlen=5)
#         time.sleep(1)

#     def callback(self, data):
#         global INTERACTING_FLAG
        
#         pub_msg_state = Int16()
#         air_pressure = data.data
#         self.recent_pressure.append(air_pressure)
#         average_of_recent_pressure = sum(self.recent_pressure) / len(self.recent_pressure)
#         print(average_of_recent_pressure)

#         if INTERACTING_FLAG == True:
#             if average_of_recent_pressure == 0.4:
#                 pub_msg_state.data = 1
#             elif average_of_recent_pressure == 1.0:
#                 pub_msg_state.data = 2
#             elif average_of_recent_pressure == 1.5:
#                 pub_msg_state.data = 3
#             self.publish_state(pub_msg_state)
#         # rospy.loginfo(f"Average Log Volume: {volume}")
        
        
#     def publish_state(self, data):
#         self.pub_state.publish(data)

class pressureSub():
    """
    気圧センサによって加えるポイントを決定する
    """
    def __init__(self):
        print("init pressure subscriber")
        # Subscriberの作成
        self.sub = rospy.Subscriber("/inflatable_touch", Bool, self.callback, queue_size=1)
        self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        # self.recent_pressure = deque([], maxlen=5)
        time.sleep(1)

    def callback(self, data):
        global INTERACTING_FLAG
        pub_msg_state = Int16()
        
        if data.data == True:
            pub_msg_state.data = min(CURRENT_UMORU_STATE + 1, 5)
        
    def publish_state(self, data):
        self.pub_state.publish(data)


class voiceSub():
    def __init__(self):
        self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        self.pub_volume = rospy.Publisher('/log_volume', Float32, queue_size=1)
        self.sub = rospy.Subscriber("/audio_volume", Float32, self.callback, queue_size=1)
        print("init voice subscriber")
        
    def low_pass_filter(self, data, alpha=0.1):
        return lfilter([1-alpha], [1, -alpha], data)
    
    def calculate_rms(self, audio_data):
        filtered_data = self.low_pass_filter(audio_data)
        rms = np.sqrt(np.mean(np.square(filtered_data)))
        return rms

    def callback(self, data):
        pub_msg_state = Int16()
        pub_msg_volume = Float32()
        print("in callback")
        rospy.sleep(0.1)
        print(f"Log Volume: {data.data}")
        volume = data.data
        if INTERACTING_FLAG == True:
            if volume < 50:
                pub_msg_state.data = 1
            elif volume >= 50 and volume < 75:
                pub_msg_state.data = 2
            elif volume >= 75 and volume < 90:
                pub_msg_state.data = 3
            elif volume >= 90:
                pub_msg_state.data = 4
            self.publish_state(pub_msg_state)

    def publish_state(self, data):
        self.pub_state.publish(data)
    
    def publish_volume(self, data):
        self.pub_volume.publish(data)

class timeController():
    """
    時間によって、stateを変える
    """
    def __init__(self):
        # self.pub_demo_state = rospy.Publisher("/demo_status", Bool, queue_size=1)
        self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        self.rate = rospy.Rate(10)  # 1Hzでpublishする設定
        self.current_time = rospy.get_time()
        # time.sleep(1)

    def spin(self):
        while not rospy.is_shutdown():
            self.check_and_publish_state()
            self.rate.sleep()
            print("TIME_CONTROLLER_LIST = ", TIME_CONTROLLER_LIST)
            print("INTERACTING_FLAG = ", INTERACTING_FLAG)
            print("CURRENT_UMORU_STATE = ", CURRENT_UMORU_STATE)

    def check_and_publish_state(self):
        global INTERACTING_FLAG
        global TIME_CONTROLLER_LIST
        global LAST_TIME
        pub_msg_state = Int16()
        pub_msg_demo_state = Bool()
        if CURRENT_UMORU_STATE in [1, 2, 3]:
            if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 10:
                pub_msg_state.data = CURRENT_UMORU_STATE + 1
                self.publish_state(pub_msg_state)
        elif CURRENT_UMORU_STATE == 4:
            if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
                pub_msg_state.data = 5
                self.publish_state(pub_msg_state)
        elif CURRENT_UMORU_STATE == 5:
            # if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
            if len(TIME_CONTROLLER_LIST) == 0 or rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
                pub_msg_state.data = 0
                pub_msg_demo_state.data = False
                self.publish_state(pub_msg_state)
                # self.publish_demo_state(pub_msg_demo_state)
                LAST_TIME = rospy.get_time()
                TIME_CONTROLLER_LIST = []
                INTERACTING_FLAG = False
    def publish_state(self, data):
        self.pub.publish(data)

    # def publish_demo_state(self, data):
    #     self.pub_demo_state.publish(data)

class umoruStateController():
    def __init__(self):
        self.pub_heart_color = rospy.Publisher("/heart_color", Float32MultiArray, queue_size=1)
        self.pub_heart_pulse = rospy.Publisher("/pulse_time", Float32, queue_size=1)
        self.pub_eye_status = rospy.Publisher("/eye_status", UInt16, queue_size=1)
        self.pub_demo_status = rospy.Publisher("/demo_status", Bool, queue_size=1)
        self.sub = self.sub = rospy.Subscriber("/umoru_state", Int16, self.callback, queue_size=1)
        print("init umoru state controller")
        
    def callback(self, data):
        pub_msg_heart_pulse = Float32()
        pub_msg_heart_color = Float32MultiArray()
        pub_msg_eye_status = UInt16()
        pub_msg_demo_status = Bool()
        global CURRENT_UMORU_STATE
        global TIME_CONTROLLER_LIST
        # print("in callback")

        if CURRENT_UMORU_STATE < int(data.data) or (CURRENT_UMORU_STATE == 5 and int(data.data) == 0):
            CURRENT_UMORU_STATE = int(data.data)
            if CURRENT_UMORU_STATE == 0:
                pub_msg_heart_pulse.data = 10
                pub_msg_heart_color.data = [0.01, 0.01, 0.01]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                print("state = 0")
            elif CURRENT_UMORU_STATE == 1:
                CURRENT_UMORU_STATE = 1
                pub_msg_heart_pulse.data = 2.0
                pub_msg_heart_color.data = [0.9,0.9,0.9]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                print("state = 1")
            elif CURRENT_UMORU_STATE == 2 and 5 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]:
                CURRENT_UMORU_STATE = 2
                pub_msg_heart_pulse.data = 1.0
                pub_msg_heart_color.data = [0.9,0.5,0.8]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                print("state = 2")
            elif (CURRENT_UMORU_STATE == 3 and 5 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]):
                CURRENT_UMORU_STATE = 3
                pub_msg_heart_pulse.data = 0.7
                pub_msg_heart_color.data = [0.9,0.3,0.5]
                pub_msg_eye_status.data = 1
                pub_msg_demo_status.data = 1
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                print("state = 3")
            elif (CURRENT_UMORU_STATE == 4 and 5 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]):
                CURRENT_UMORU_STATE = 4
                pub_msg_heart_pulse.data = 0.5
                pub_msg_heart_color.data = [0.9,0,0]
                pub_msg_demo_status.data = 1
                pub_msg_eye_status.data = 1
                arm_client.reset_pose()
                arm_client.hug()
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                print("state = 4")
            elif CURRENT_UMORU_STATE == 5 and 5 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]:
                CURRENT_UMORU_STATE = 5
                arm_client.reset_pose()
                arm_client.init_pose()
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                pub_msg_eye_status.data = 1
                pub_msg_demo_status.data = 1
                print("state =======================================5==========================")
            self.publish_demo_status(pub_msg_demo_status)
            self.publish_heart_pulse(pub_msg_heart_pulse)
            self.publish_heart_color(pub_msg_heart_color)
            self.publish_eye_status(pub_msg_eye_status)
            
    def publish_heart_pulse(self, data):
        self.pub_heart_pulse.publish(data)

    def publish_heart_color(self, data):
        self.pub_heart_color.publish(data)
    
    def publish_eye_status(self, data):
        self.pub_eye_status.publish(data)

    def publish_demo_status(self, data):
        self.pub_demo_status.publish(data)
    


if __name__ == '__main__':
    print("hoge")
    rospy.init_node("test_node")
    arm_client = MotionClient("both")
    # arm_client = MotionClient("rarm")
    # arm_client = MotionClient("right")
    arm_client.init_pose()
    node0 = pressureSub()
    node1 = voiceSub()
    node2 = umoruStateController()
    node3 = startAndEndFlag()
    time_controller = timeController()  # timeControllerのインスタンスを作成

    # time_controllerのspinメソッドを実行
    time_controller.spin()  
#     while not rospy.is_shutdown():
#         rospy.sleep(0.1)
