#!/usr/bin/env python3.8
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
import random
from collections import deque
from scipy.signal import lfilter
from playsound import playsound
import simpleaudio
import threading

# global CURRENT_UMORU_STATE, INTERACTING_FLAG, FACE_FIND_FLAG, TIME_CONTROLLER_LIST, LAST_TIME, USE_ARM
# CURRENT_UMORU_STATE = 0
# INTERACTING_FLAG = False
# FACE_FIND_FLAG = False
# TIME_CONTROLLER_LIST = []
# LAST_TIME = None
# USE_ARM = True

# class startAndEndFlag():
#     """
#     interactionの始まりを規定するクラス
#     """
#     def __init__(self):
#         print("init start and end flag")
#         # self.sub = rospy.Subscriber("/camera/face_skeleton_with_depth/output/pose", HumanSkeletonArray, self.callback, queue_size=1)
#         self.sub = rospy.Subscriber("/camera/color/face_pose_estimation/output/skeleton", HumanSkeletonArray, self.callback, queue_size=1)
#         self.sub_emergency = rospy.Subscriber("/if_camera_doesnot_work", Bool, self.callback_for_emergency)
#         self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
#         self.pub_demo = rospy.Publisher('/demo_status', Bool, queue_size=1)
#         self.face_appeared_time = None
        
#     def callback(self, data):
#         global FACE_FIND_FLAG
#         global INTERACTING_FLAG
#         global TIME_CONTROLLER_LIST
#         global LAST_TIME
#         pub_msg_state = Int16()
#         pub_msg_demo_status = Bool()
#         skeletons = data.skeletons
#         # もし体験中じゃない かつ 前回の体験から10秒以上経過しているとき
#         if INTERACTING_FLAG == False and (LAST_TIME == None or rospy.get_time() - LAST_TIME >= 10):
#             # print("INTERACTING_FLAS is ", INTERACTING_FLAG)
#             # print("FACE FIND FLAG is ", FACE_FIND_FLAG)
#             if len(skeletons) == 0:
#                 # 顔が見えない場合
#                 FACE_FIND_FLAG = False
#             elif len(skeletons) != 0:
#                 # 顔が見える場合
#                 if FACE_FIND_FLAG == True:
#                     if rospy.get_time() - self.face_appeared_time >= 3:
#                         # もし顔が見えてから3秒経過した場合、体験スタート
#                         INTERACTING_FLAG = True
#                         print("================= 1. Interaction Start (by camera)  ======================")
#                         TIME_CONTROLLER_LIST = []
#                         TIME_CONTROLLER_LIST.append(rospy.get_time())
#                         pub_msg_state.data = 1
#                         self.publish(pub_msg_state)
#                         FACE_FIND_FLAG = False
#                         pub_msg_demo_status.data = 0
#                         self.publish_demo(pub_msg_demo_status)
#                         # playsound("sounds/umoru-first-phrase.mp3")
#                         # time.sleep(10)
#                 else:
#                     # 顔が始めて見えたとき、その時刻を記録する
#                     self.face_appeared_time = rospy.get_time()
#                     FACE_FIND_FLAG = True

#     def callback_for_emergency(self, data):
#         # もしカメラが完全に落ちたときの処理
#         # /if_camera_doesnot_workになにかpublishすると体験スタートにできる
#         global FACE_FIND_FLAG
#         global INTERACTING_FLAG
#         global TIME_CONTROLLER_LIST
#         pub_msg_state = Int16()
#         if INTERACTING_FLAG == False and (LAST_TIME == None or rospy.get_time() - LAST_TIME >= 10):
#             INTERACTING_FLAG = True
#             TIME_CONTROLLER_LIST = []
#             TIME_CONTROLLER_LIST.append(rospy.get_time())
#             pub_msg_state.data = 1
#             self.publish(pub_msg_state)
#             FACE_FIND_FLAG = False
#             print("================= 1. Interaction Start (by force!!!)  ======================")

#     def publish(self, data):
#         self.pub.publish(data)

#     def publish_demo(self, data):
#         self.pub_demo.publish(data)

# # class pressureSub():
# #     """
# #     気圧センサによって加えるポイントを決定する
# #     """
# #     def __init__(self):
# #         print("init pressure subscriber")
# #         # Subscriberの作成
# #         self.sub = rospy.Subscriber("/sensor", Int8, self.callback, queue_size=1)
# #         self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
# #         self.recent_pressure = deque([], maxlen=5)
# #         time.sleep(1)

# #     def callback(self, data):
# #         global INTERACTING_FLAG
        
# #         pub_msg_state = Int16()
# #         air_pressure = data.data
# #         self.recent_pressure.append(air_pressure)
# #         average_of_recent_pressure = sum(self.recent_pressure) / len(self.recent_pressure)
# #         print(average_of_recent_pressure)

# #         if INTERACTING_FLAG == True:
# #             if average_of_recent_pressure == 0.4:
# #                 pub_msg_state.data = 1
# #             elif average_of_recent_pressure == 1.0:
# #                 pub_msg_state.data = 2
# #             elif average_of_recent_pressure == 1.5:
# #                 pub_msg_state.data = 3
# #             self.publish_state(pub_msg_state)
# #         # rospy.loginfo(f"Average Log Volume: {volume}")
        
# #     def publish_state(self, data):
# #         self.pub_state.publish(data)

# class pressureSub():
#     """
#     気圧センサによってumoru_stateを変える
#     具体的には、押された（inflatable_touchにtrueがきた）ら、次のUMORU_STATEをpublishする
#     """
#     def __init__(self):
#         print("init pressure subscriber")
#         # Subscriberの作成
#         self.sub = rospy.Subscriber("/inflatable_touch", Bool, self.callback, queue_size=1)
#         self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
#         # self.recent_pressure = deque([], maxlen=5)
#         time.sleep(1)

#     def callback(self, data):
#         global INTERACTING_FLAG
#         pub_msg_state = Int16()
        
#         if INTERACTING_FLAG == True and data.data == True:
#             pub_msg_state.data = min(CURRENT_UMORU_STATE + 1, 5)
#             self.publish_state(pub_msg_state)
#             print("~~~~~~~~~~~~~~~ Trigger : inflatable pressure  ~~~~~~~~~~~~~~~~~~~~~")
            
#     def publish_state(self, data):
#         self.pub_state.publish(data)


# class voiceSub():
#     """
#     周囲の音量によってumoru_stateを変える
#     具体的には音量が70を変えると次のstateに変化する
#     """
#     def __init__(self):
#         self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
#         self.pub_volume = rospy.Publisher('/log_volume', Float32, queue_size=1)
#         self.sub = rospy.Subscriber("/audio_volume", Float32, self.callback, queue_size=1)
#         print("init voice subscriber")
        
#     # def low_pass_filter(self, data, alpha=0.1):
#     #     return lfilter([1-alpha], [1, -alpha], data)
    
#     # def calculate_rms(self, audio_data):
#     #     filtered_data = self.low_pass_filter(audio_data)
#     #     rms = np.sqrt(np.mean(np.square(filtered_data)))
#     #     return rms

#     def callback(self, data):
#         pub_msg_state = Int16()
#         pub_msg_volume = Float32()
#         rospy.sleep(0.1)
#         volume = data.data
#         # if INTERACTING_FLAG == True:
#         #     if volume < 50:
#         #         pub_msg_state.data = 1
#         #     elif volume >= 50 and volume < 75:
#         #         pub_msg_state.data = 2
#         #     elif volume >= 75 and volume < 90:
#         #         pub_msg_state.data = 3
#         #     elif volume >= 90:
#         #         pub_msg_state.data = 4
#         #     self.publish_state(pub_msg_state)
#         if INTERACTING_FLAG == True:
#             if volume > 70:
#                 pub_msg_state.data = min(CURRENT_UMORU_STATE + 1, 5)
#                 self.publish_state(pub_msg_state)
        
#     def publish_state(self, data):
#         self.pub_state.publish(data)
    
#     # def publish_volume(self, data):
#     #     self.pub_volume.publish(data)

# class timeController():
#     """
#     時間によって、stateを変える
#     """
#     def __init__(self):
#         self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
#         self.rate = rospy.Rate(10)  # 1Hzでpublishする設定
#         self.current_time = rospy.get_time()

#     def spin(self):
#         while not rospy.is_shutdown():
#             self.check_and_publish_state()
#             self.rate.sleep()
#             # print("TIME_CONTROLLER_LIST = ", TIME_CONTROLLER_LIST)
#             # # print("INTERACTING_FLAG = ", INTERACTING_FLAG)
#             # print("CURRENT_UMORU_STATE = ", CURRENT_UMORU_STATE)

#     def check_and_publish_state(self):
#         global INTERACTING_FLAG
#         global TIME_CONTROLLER_LIST
#         global LAST_TIME
#         global PUBLISHED_TIME_OVER
#         pub_msg_state = Int16()
#         pub_msg_demo_state = Bool()
#         # もしstateが1, 2, 3のいずれかの場合は10秒以上はそのstateにとどまる
#         if CURRENT_UMORU_STATE in [1, 2, 3]:
#             if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
#                 pub_msg_state.data = CURRENT_UMORU_STATE + 1
#                  #print("~~~~~~~~~~~~~~~ Trigger : Time Over A  ~~~~~~~~~~~~~~~~~~~~~")
#                 self.publish_state(pub_msg_state)
#         # もしstateが4の場合は20秒以上はそのstateにとどまる
#         elif CURRENT_UMORU_STATE == 4:
#             if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
#                 pub_msg_state.data = 5
#                 self.publish_state(pub_msg_state)
#                 # print("~~~~~~~~~~~~~~~ Trigger : Time Over B ~~~~~~~~~~~~~~~~~~~~~")
#             # もしstateが5の場合は20秒以上はそのstateにとどまる
#         elif CURRENT_UMORU_STATE == 5:
#             if len(TIME_CONTROLLER_LIST) == 0 or rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
#                 pub_msg_state.data = 0
#                 pub_msg_demo_state.data = False
#                 self.publish_state(pub_msg_state)
#                 LAST_TIME = rospy.get_time()
#                 TIME_CONTROLLER_LIST = []
#                 INTERACTING_FLAG = False
#                 # print("~~~~~~~~~~~~~~~ Trigger : Time Over C ~~~~~~~~~~~~~~~~~~~~~")
#     def publish_state(self, data):
#         self.pub.publish(data)

# # class timeController():
# #     """
# #     時間によって、stateを変える
# #     """
# #     def __init__(self):
# #         self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
# #         self.rate = rospy.Rate(10)  # 1Hzでpublishする設定
# #         self.current_time = rospy.get_time()
# #         self.previous_state = None  # 前回の状態を追跡

# #     def spin(self):
# #         while not rospy.is_shutdown():
# #             self.check_and_publish_state()
# #             self.rate.sleep()

# #     def check_and_publish_state(self):
# #         global INTERACTING_FLAG
# #         global TIME_CONTROLLER_LIST
# #         global LAST_TIME
# #         pub_msg_state = Int16()
# #         pub_msg_demo_state = Bool()
        
# #         # 状態を変更するためのフラグ
# #         state_changed = False  

# #         if CURRENT_UMORU_STATE in [1, 2, 3]:
# #             if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 10:
# #                 pub_msg_state.data = CURRENT_UMORU_STATE + 1
# #                 state_changed = True
# #                 TIME_CONTROLLER_LIST.append(rospy.get_time())
                
# #         elif CURRENT_UMORU_STATE == 4:
# #             if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
# #                 pub_msg_state.data = 5
# #                 state_changed = True
# #                 TIME_CONTROLLER_LIST.append(rospy.get_time())
                
# #         elif CURRENT_UMORU_STATE == 5:
# #             if len(TIME_CONTROLLER_LIST) == 0 or rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
# #                 pub_msg_state.data = 0
# #                 pub_msg_demo_state.data = False
# #                 state_changed = True
# #                 TIME_CONTROLLER_LIST = []
# #                 INTERACTING_FLAG = False
# #                 LAST_TIME = rospy.get_time()
        
# #         # 状態が変わり、前回の状態と異なる場合のみpublishする
# #         if state_changed and pub_msg_state.data != self.previous_state:
# #             self.previous_state = pub_msg_state.data  # 状態を更新
# #             self.publish_state(pub_msg_state)
# #             print(f"State updated to {pub_msg_state.data}")
        
# #     def publish_state(self, data):
# #         self.pub.publish(data)


# class umoruStateController():
#     def __init__(self):
#         self.pub_heart_color = rospy.Publisher("/heart_color", Float32MultiArray, queue_size=1)
#         self.pub_heart_pulse = rospy.Publisher("/pulse_time", Float32, queue_size=1)
#         self.pub_eye_status = rospy.Publisher("/eye_status", UInt16, queue_size=1)
#         self.pub_demo_status = rospy.Publisher("/demo_status", Bool, queue_size=1)
#         self.sub = rospy.Subscriber("/umoru_state", Int16, self.callback, queue_size=1)
#         print("init umoru state controller")
        
#     def play_sound_async(self, file_path):
#         threading.Thread(target=playsound, args=(file_path,)).start()
    
#     def callback(self, data):
#         pub_msg_heart_pulse = Float32()
#         pub_msg_heart_color = Float32MultiArray()
#         pub_msg_eye_status = UInt16()
#         pub_msg_demo_status = Bool()
#         global CURRENT_UMORU_STATE
#         global TIME_CONTROLLER_LIST
#         global USE_ARM
#         # print("in callback")
        
#         if CURRENT_UMORU_STATE < int(data.data) or (CURRENT_UMORU_STATE == 5 and int(data.data) == 0):
#             CURRENT_UMORU_STATE = CURRENT_UMORU_STATE + 1
#             if CURRENT_UMORU_STATE == 0:
#                 pub_msg_heart_pulse.data = 10
#                 pub_msg_heart_color.data = [0.01, 0.01, 0.01]
#                 pub_msg_eye_status.data = 3
#                 pub_msg_demo_status.data = false
#                 self.publish_heart_pulse(pub_msg_heart_pulse)
#                 self.publish_heart_color(pub_msg_heart_color)
#                 self.publish_eye_status(pub_msg_eye_status)
#                 self.publish_demo_status(pub_msg_demo_status)
#                 print("============== state = 0 (interaction was reset)  ===================")
#             elif CURRENT_UMORU_STATE == 1:
#                 CURRENT_UMORU_STATE = 1
#                 pub_msg_heart_pulse.data = 2.0
#                 pub_msg_heart_color.data = [0.9,0.9,0.9]
#                 pub_msg_eye_status.data = 3
#                 pub_msg_demo_status.data = false
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
#                 self.play_sound_async("/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-first-phrase.wav")
#                 time_to_sleep = 10
#                 print("=============== state = 1 (heart appeared) ========================")
#             elif CURRENT_UMORU_STATE == 2 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]:
#                 CURRENT_UMORU_STATE = 2
#                 pub_msg_heart_pulse.data = 1
#                 pub_msg_heart_color.data = [0.9,0.5,0.8]
#                 pub_msg_eye_status.data = 3
#                 pub_msg_demo_status.data = 0
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
#                 self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-yobimashitaka.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-bikkuri.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-iinioi.wav"]))
#                 print("=============== state = 2 ========================")
#             elif (CURRENT_UMORU_STATE == 3 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]):
#                 CURRENT_UMORU_STATE = 3
#                 pub_msg_heart_pulse.data = 0.8
#                 pub_msg_heart_color.data = [0.9,0.3,0.5]
#                 pub_msg_eye_status.data = 1
#                 pub_msg_demo_status.data = 1
#                 self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-nukumori.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-kodou.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-kokyu-fukkatsu.wav"]))
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
#                 print("=============== state = 3 ========================")
#                 print("=============== 呼吸スタート =====================")
#             elif (CURRENT_UMORU_STATE == 4 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]):
#                 print("aaaaaaaaaaaaaaaaaaaaaaa")
#                 CURRENT_UMORU_STATE = 4
#                 pub_msg_heart_pulse.data = 0.6
#                 pub_msg_heart_color.data = [0.9,0,0]
#                 pub_msg_demo_status.data = 1
#                 pub_msg_eye_status.data = 1
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
#                 if USE_ARM == True:
#                     arm_client.reset_pose()
#                     arm_client.hug()
#                 self.play_sound_async("/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-hug-fukkatsu.wav")
#                 print("=============== state = 4 ========================")
#                 print("=============== ハグスタート ======================")
#             elif CURRENT_UMORU_STATE == 5 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]:
#                 CURRENT_UMORU_STATE = 5
#                 self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-thankyou-pattern1.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-thankyou-pattern2.wav"]))
#                 if USE_ARM == True:
#                     arm_client.reset_pose()
#                     arm_client.init_pose()
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
#                 pub_msg_eye_status.data = 1
#                 pub_msg_demo_status.data = 1
#                 pub_msg_heart_pulse.data = 0.4
#                 print("=============== state = 5 ========================")
#                 print("=============== ハグ終了 ======================")
#                 # 最初の状態にリセット
#                 pub_msg_state = Int16()
#                 pub_msg_state.data = 0
#                 self.publish_state(pub_msg_state)


#             self.publish_demo_status(pub_msg_demo_status)
#             if pub_msg_heart_pulse.data != 0:
#                 self.publish_heart_pulse(pub_msg_heart_pulse)
#                 time.sleep(0.1)
#             self.publish_heart_color(pub_msg_heart_color)
#             self.publish_eye_status(pub_msg_eye_status)
            
#     def publish_heart_pulse(self, data):
#         self.pub_heart_pulse.publish(data)

#     def publish_heart_color(self, data):
#         self.pub_heart_color.publish(data)
    
#     def publish_eye_status(self, data):
#         self.pub_eye_status.publish(data)

#     def publish_demo_status(self, data):
#         self.pub_demo_status.publish(data)


# if __name__ == '__main__':
#     rospy.init_node("umoru_main")
#     if USE_ARM == True:
#         arm_client = MotionClient("both")
#         arm_client.init_pose()
#     node0 = pressureSub()
#     node1 = voiceSub()
#     node2 = umoruStateController()
#     node3 = startAndEndFlag()
#     time_controller = timeController()  # timeControllerのインスタンスを作成

#     # time_controllerのspinメソッドを実行
#     time_controller.spin()  
# #     while not rospy.is_shutdown():
# #         rospy.sleep(0.1)

#!/usr/bin/env python3.8
# import rospy
# import time
# from std_msgs.msg import String, Bool, Float32, Float32MultiArray, Int8, Int16, UInt16
# from jsk_recognition_msgs.msg import HumanSkeletonArray
# from playsound import playsound
# import threading

# グローバル変数
global CURRENT_UMORU_STATE, INTERACTING_FLAG, FACE_FIND_FLAG, TIME_CONTROLLER_LIST, LAST_TIME, USE_ARM
CURRENT_UMORU_STATE = 0
INTERACTING_FLAG = False
FACE_FIND_FLAG = False
TIME_CONTROLLER_LIST = []
LAST_TIME = None
USE_ARM = True

class startAndEndFlag():
    def __init__(self):
        print("init start and end flag")
        self.sub = rospy.Subscriber("/camera/color/face_pose_estimation/output/skeleton", HumanSkeletonArray, self.callback, queue_size=1)
        self.sub_emergency = rospy.Subscriber("/if_camera_doesnot_work", Bool, self.callback_for_emergency)
        self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        self.pub_demo_arduino = rospy.Publisher('/demo_status', Bool, queue_size=1)
        self.face_appeared_time = None

    def callback(self, data):
        global FACE_FIND_FLAG, INTERACTING_FLAG, TIME_CONTROLLER_LIST, LAST_TIME
        pub_msg_state = Int16()
        pub_msg_demo_status_arduino = Bool()  # Arduino向けのdemo_status
        skeletons = data.skeletons

        if INTERACTING_FLAG == False and (LAST_TIME is None or rospy.get_time() - LAST_TIME >= 10):
            if len(skeletons) == 0:
                FACE_FIND_FLAG = False
            elif len(skeletons) != 0:
                if FACE_FIND_FLAG:
                    if rospy.get_time() - self.face_appeared_time >= 3:
                        INTERACTING_FLAG = True
                        print("================= 1. Interaction Start (by camera)  ======================")
                        TIME_CONTROLLER_LIST = [rospy.get_time()]
                        pub_msg_state.data = 1
                        self.publish(pub_msg_state)
                        FACE_FIND_FLAG = False
                        pub_msg_demo_status_arduino.data = 0
                        self.publish_demo(pub_msg_demo_status_arduino)
                else:
                    self.face_appeared_time = rospy.get_time()
                    FACE_FIND_FLAG = True

    def callback_for_emergency(self, data):
        global FACE_FIND_FLAG, INTERACTING_FLAG, TIME_CONTROLLER_LIST, LAST_TIME
        pub_msg_state = Int16()
        if INTERACTING_FLAG == False and (LAST_TIME is None or rospy.get_time() - LAST_TIME >= 10):
            INTERACTING_FLAG = True
            TIME_CONTROLLER_LIST = [rospy.get_time()]
            pub_msg_state.data = 1
            self.publish(pub_msg_state)
            FACE_FIND_FLAG = False
            print("================= 1. Interaction Start (by force!!!)  ======================")

    def publish(self, data):
        self.pub.publish(data)

    def publish_demo(self, data):
        self.pub_demo_arduino.publish(data)


class pressureSub():
    def __init__(self):
        print("init pressure subscriber")
        self.sub = rospy.Subscriber("/inflatable_touch", Bool, self.callback, queue_size=1)
        self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        time.sleep(1)

    def callback(self, data):
        global INTERACTING_FLAG, CURRENT_UMORU_STATE
        pub_msg_state = Int16()

        if INTERACTING_FLAG and data.data:
            pub_msg_state.data = min(CURRENT_UMORU_STATE + 1, 5)
            self.publish_state(pub_msg_state)
            print("~~~~~~~~~~~~~~~ Trigger : inflatable pressure  ~~~~~~~~~~~~~~~~~~~~~")
            
    def publish_state(self, data):
        self.pub_state.publish(data)


class timeController():
    def __init__(self):
        self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        self.rate = rospy.Rate(10)
        self.current_time = rospy.get_time()

    def spin(self):
        while not rospy.is_shutdown():
            self.check_and_publish_state()
            self.rate.sleep()

    def check_and_publish_state(self):
        global INTERACTING_FLAG, TIME_CONTROLLER_LIST, LAST_TIME, CURRENT_UMORU_STATE
        pub_msg_state = Int16()
        pub_msg_demo_state = Bool()

        if CURRENT_UMORU_STATE in [1, 2, 3] and rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
            pub_msg_state.data = CURRENT_UMORU_STATE + 1
            self.publish_state(pub_msg_state)
            TIME_CONTROLLER_LIST.append(rospy.get_time())
        elif CURRENT_UMORU_STATE == 4 and rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
            pub_msg_state.data = 5
            self.publish_state(pub_msg_state)
            TIME_CONTROLLER_LIST.append(rospy.get_time())
        elif CURRENT_UMORU_STATE == 5 and (len(TIME_CONTROLLER_LIST) == 0 or rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20):
            pub_msg_state.data = 0
            pub_msg_demo_state.data = False
            self.publish_state(pub_msg_state)
            LAST_TIME = rospy.get_time()
            TIME_CONTROLLER_LIST = []
            INTERACTING_FLAG = False

    def publish_state(self, data):
        self.pub.publish(data)


class umoruStateController():
    def __init__(self):
        self.pub_heart_color = rospy.Publisher("/heart_color", Float32MultiArray, queue_size=1)
        self.pub_heart_pulse = rospy.Publisher("/pulse_time", Float32, queue_size=1)
        self.pub_eye_status = rospy.Publisher("/eye_status", UInt16, queue_size=1)
        self.pub_demo_status_internal = rospy.Publisher("/demo_status_internal", Bool, queue_size=1)  # 内部確認用
        self.pub_demo_status_arduino = rospy.Publisher("/demo_status", Bool, queue_size=1)  # Arduinoに送信するトピック
        self.sub = rospy.Subscriber("/umoru_state", Int16, self.callback, queue_size=1)
        print("init umoru state controller")
        
    def play_sound_async(self, file_path):
        threading.Thread(target=playsound, args=(file_path,)).start()

    def callback(self, data):
        global CURRENT_UMORU_STATE, TIME_CONTROLLER_LIST
        pub_msg_heart_pulse = Float32()
        pub_msg_heart_color = Float32MultiArray()
        pub_msg_eye_status = UInt16()
        pub_msg_demo_status_internal = Bool()
        pub_msg_demo_status_arduino = Bool()  # Arduino向けのデモ状態

        if CURRENT_UMORU_STATE < int(data.data) or (CURRENT_UMORU_STATE == 5 and int(data.data) == 0):
            CURRENT_UMORU_STATE = CURRENT_UMORU_STATE + 1
            print(f"=============== state = {CURRENT_UMORU_STATE} ========================")

            # 呼吸開始時（state == 3）にArduino向けにdemo_status Trueを送信
            if CURRENT_UMORU_STATE == 3:
                pub_msg_demo_status_arduino.data = True
                self.publish_demo_status_arduino(pub_msg_demo_status_arduino)
                print("=============== 呼吸開始 (demo_status to Arduino: True) ========================")

            # デモが終了する際（state == 5）に、Arduino向けにdemo_status Falseを送信し、内部もリセット
            if CURRENT_UMORU_STATE == 5:
                print("=============== Ending state 5, resetting to state 0 ========================")
                CURRENT_UMORU_STATE = 0  # 状態0に戻す
                TIME_CONTROLLER_LIST.clear()  # タイムリストもクリア
                INTERACTING_FLAG = False  # 体験フラグをリセット
                LAST_TIME = rospy.get_time()  # 最後の時間を更新

                # 初期状態に戻るためのパラメータを設定
                pub_msg_heart_pulse.data = 10
                pub_msg_heart_color.data = [0.01, 0.01, 0.01]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status_internal.data = False  # 内部デモフラグをFalse
                pub_msg_demo_status_arduino.data = False   # ArduinoにもFalseを送信

            # 各状態に応じたパブリッシュを行う
            self.publish_demo_status_internal(pub_msg_demo_status_internal)
            self.publish_demo_status_arduino(pub_msg_demo_status_arduino)
            self.publish_heart_pulse(pub_msg_heart_pulse)
            self.publish_heart_color(pub_msg_heart_color)
            self.publish_eye_status(pub_msg_eye_status)

    def publish_heart_pulse(self, data):
        self.pub_heart_pulse.publish(data)

    def publish_heart_color(self, data):
        self.pub_heart_color.publish(data)
    
    def publish_eye_status(self, data):
        self.pub_eye_status.publish(data)

    def publish_demo_status_internal(self, data):
        self.pub_demo_status_internal.publish(data)

    def publish_demo_status_arduino(self, data):
        self.pub_demo_status_arduino.publish(data)


if __name__ == '__main__':
    rospy.init_node("umoru_main")
    if USE_ARM == True:
        arm_client = MotionClient("both")
        arm_client.init_pose()
    node0 = pressureSub()
    node1 = timeController()
    node2 = umoruStateController()
    node3 = startAndEndFlag()
    
    # time_controllerのspinメソッドを実行
    node1.spin()
