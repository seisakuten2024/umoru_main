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

global CURRENT_UMORU_STATE, INTERACTING_FLAG, FACE_FIND_FLAG, TIME_CONTROLLER_LIST, LAST_TIME, USE_ARM
CURRENT_UMORU_STATE = 0
INTERACTING_FLAG = False
FACE_FIND_FLAG = False
TIME_CONTROLLER_LIST = []
LAST_TIME = None
USE_ARM = True

class startAndEndFlag():
    """
    interactionの始まりを規定するクラス
    """
    def __init__(self):
        print("init start and end flag")
        self.sub = rospy.Subscriber("/camera/color/face_pose_estimation/output/skeleton", HumanSkeletonArray, self.callback, queue_size=3)
        self.sub_emergency = rospy.Subscriber("/if_camera_doesnot_work", Bool, self.callback_for_emergency)
        self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=10)
        self.pub_change_trigger = rospy.Publisher('/change_trigger', String, queue_size=10)
        self.pub_demo = rospy.Publisher('/demo_status', Bool, queue_size=10)
        self.face_appeared_time = None
        rospy.sleep(0.1)
        
    def callback(self, data):
        global FACE_FIND_FLAG
        global INTERACTING_FLAG
        global TIME_CONTROLLER_LIST
        global LAST_TIME
        pub_msg_state = Int16()
        pub_msg_demo_status = Bool()
        pub_msg_change_trigger = String()
        skeletons = data.skeletons
        # もし体験中じゃない かつ 前回の体験から10秒以上経過しているとき
        if INTERACTING_FLAG == False and (LAST_TIME == None or rospy.get_time() - LAST_TIME >= 10):
            if len(skeletons) == 0:
                # 顔が見えない場合
                FACE_FIND_FLAG = False
            elif len(skeletons) != 0:
                # 顔が見える場合
                if FACE_FIND_FLAG == True:
                    if rospy.get_time() - self.face_appeared_time >= 3:
                        # もし顔が見えてから3秒経過した場合、体験スタート
                        INTERACTING_FLAG = True
                        print("================= 1. Interaction Start (by camera)  ======================")
                        TIME_CONTROLLER_LIST = []
                        # TIME_CONTROLLER_LIST.append(rospy.get_time())
                        # pub_msg_state.data = 1
                        # self.publish(pub_msg_state)
                        pub_msg_change_trigger.data = "face"
                        self.publish_change_trigger(pub_msg_change_trigger)
                        FACE_FIND_FLAG = False
                        pub_msg_demo_status.data = 0
                        self.publish_demo(pub_msg_demo_status)
                else:
                    # 顔が始めて見えたとき、その時刻を記録する
                    self.face_appeared_time = rospy.get_time()
                    FACE_FIND_FLAG = True
        rospy.sleep(0.05)

    def callback_for_emergency(self, data):
        # もしカメラが完全に落ちたときの処理
        # /if_camera_doesnot_workになにかpublishすると体験スタートにできる
        global FACE_FIND_FLAG
        global INTERACTING_FLAG
        global TIME_CONTROLLER_LIST
        pub_msg_state = Int16()
        pub_msg_change_trigger = String()
        if INTERACTING_FLAG == False and (LAST_TIME == None or rospy.get_time() - LAST_TIME >= 10):
            INTERACTING_FLAG = True
            pub_msg_state.data = 1
            pub_msg_change_trigger.data = "face"
            self.publish(pub_msg_state)
            self.publish_change_trigger(pub_msg_change_trigger)
            FACE_FIND_FLAG = False
            print("================= 1. Interaction Start (by force!!!)  ======================")
        rospy.sleep(0.05)

    def publish(self, data):
        self.pub.publish(data)

    def publish_demo(self, data):
        self.pub_demo.publish(data)

    def publish_change_trigger(self, data):
        self.pub_change_trigger.publish(data)

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
    気圧センサによってumoru_stateを変える
    具体的には、押された（inflatable_touchにtrueがきた）ら、次のUMORU_STATEをpublishする
    """
    def __init__(self):
        print("init pressure subscriber")
        # Subscriberの作成
        self.sub = rospy.Subscriber("/inflatable_touch", Bool, self.callback, queue_size=10)
        self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=10)
        self.pub_change_trigger = rospy.Publisher('/change_trigger', String, queue_size=10)
        # self.recent_pressure = deque([], maxlen=5)
        rospy.sleep(0.1)

    def callback(self, data):
        global INTERACTING_FLAG
        global CURRENT_UMORU_STATE
        pub_msg_state = Int16()
        pub_msg_change_trigger = String()
        # if INTERACTING_FLAG == True and data.data == True and CURRENT_UMORU_STATE < 4:
        if INTERACTING_FLAG == True and data.data == True and 1 <= CURRENT_UMORU_STATE <= 3:
            pub_msg_state.data = min(CURRENT_UMORU_STATE + 1, 5)
            pub_msg_change_trigger = "inflatable"
            self.publish_state(pub_msg_state)
            self.publish_change_trigger(pub_msg_change_trigger)
            # print("~~~~~~~~~~~~~~~ Trigger : inflatable pressure  ~~~~~~~~~~~~~~~~~~~~~")
        rospy.sleep(0.05)

    def publish_state(self, data):
        self.pub_state.publish(data)

    def publish_change_trigger(self, data):
        self.pub_change_trigger.publish(data)


class voiceSub():
    """
    周囲の音量によってumoru_stateを変える
    具体的には音量が70を変えると次のstateに変化する
    """
    def __init__(self):
        self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=10)
        self.sub = rospy.Subscriber("/audio_volume", Float32, self.callback, queue_size=1)
        self.pub_change_trigger = rospy.Publisher('/change_trigger', String, queue_size=10)
        print("init voice subscriber")
        rospy.sleep(0.1)

    def callback(self, data):
        pub_msg_state = Int16()
        pub_msg_change_trigger = String()
        # rospy.sleep(0.1)
        volume = data.data
        # if INTERACTING_FLAG == True:
        #     if volume < 50:
        #         pub_msg_state.data = 1
        #     elif volume >= 50 and volume < 75:
        #         pub_msg_state.data = 2
        #     elif volume >= 75 and volume < 90:
        #         pub_msg_state.data = 3
        #     elif volume >= 90:
        #         pub_msg_state.data = 4
        #     self.publish_state(pub_msg_state)
        if INTERACTING_FLAG == True and volume > 70 and 1 <= CURRENT_UMORU_STATE <= 3:
            pub_msg_state.data = min(CURRENT_UMORU_STATE + 1, 5)
            pub_msg_change_trigger.data = "voice"
            self.publish_state(pub_msg_state)
            self.publish_change_trigger(pub_msg_change_trigger)
        rospy.sleep(0.05)
        
    def publish_state(self, data):
        self.pub_state.publish(data)

    def publish_change_trigger(self, data):
        self.pub_change_trigger.publish(data)

class timeController():
    """
    時間によって、stateを変える
    """
    def __init__(self):
        self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=10)
        self.pub_change_trigger =rospy.Publisher("/change_trigger", String, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hzでpublishする設定
        # self.current_time = rospy.get_time()
        rospy.sleep(0.1)
        
    def spin(self):
        while not rospy.is_shutdown():
            self.check_and_publish_state()
            self.rate.sleep()
            print("TIME_CONTROLLER_LIST = ", TIME_CONTROLLER_LIST)
            # # print("INTERACTING_FLAG = ", INTERACTING_FLAG)
            print("CURRENT_UMORU_STATE = ", CURRENT_UMORU_STATE)
            print("LAST_TIME = ", LAST_TIME)
            print("INTERACTING_FLAG = ", INTERACTING_FLAG)

    def check_and_publish_state(self):
        global INTERACTING_FLAG
        global TIME_CONTROLLER_LIST
        global LAST_TIME
        global PUBLISHED_TIME_OVER
        global CURRENT_UMORU_STATE
        pub_msg_state = Int16()
        pub_msg_demo_state = Bool()
        pub_msg_change_trigger = String()
        if 1 <= CURRENT_UMORU_STATE <= 5:
            if rospy.get_time() - TIME_CONTROLLER_LIST[CURRENT_UMORU_STATE - 1] > 20:
                pub_msg_change_trigger.data = "time"
                self.publish_change_trigger(pub_msg_change_trigger)
                
        # # もしstateが1, 2, 3のいずれかの場合は10秒以上はそのstateにとどまる
        # if CURRENT_UMORU_STATE in [1, 2, 3]:
        #     if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 10:
        #         pub_msg_state.data = CURRENT_UMORU_STATE + 1
        #          #print("~~~~~~~~~~~~~~~ Trigger : Time Over A  ~~~~~~~~~~~~~~~~~~~~~")
        #         self.publish_state(pub_msg_state)
        # # もしstateが4の場合は20秒以上はそのstateにとどまる
        # elif CURRENT_UMORU_STATE == 4:
        #     if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 10:
        #         pub_msg_state.data = 5
        #         self.publish_state(pub_msg_state)
        #         print("~~~~~~~~~~~~~~~ Trigger : Time Over B ~~~~~~~~~~~~~~~~~~~~")
        #     print("state 4 end")
        #     # もしstateが5の場合は20秒以上はそのstateにとどまる
        # elif CURRENT_UMORU_STATE == 5:
        #     print("check state 5")
        #     if len(TIME_CONTROLLER_LIST) == 0 or rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 10:
        #         pub_msg_state.data = 0
        #         pub_msg_demo_state.data = False
        #         self.publish_state(pub_msg_state)
        #         LAST_TIME = rospy.get_time()
        #         TIME_CONTROLLER_LIST = []
        #         INTERACTING_FLAG = False
        #         print(f"check state 5: {CURRENT_UMORU_STATE}")
        #         print("~~~~~~~~~~~~~~~ Trigger : Time Over C ~~~~~~~~~~~~~~~~~~~~~")
        rospy.sleep(0.05)

    def publish_state(self, data):
        self.pub.publish(data)

    def publish_change_trigger(self, data):
        self.pub_change_trigger.publish(data)

# class timeController():
#     """
#     時間によって、stateを変える
#     """
#     def __init__(self):
#         self.pub = rospy.Publisher('/umoru_state', Int16, queue_size=1)
#         self.rate = rospy.Rate(10)  # 1Hzでpublishする設定
#         self.current_time = rospy.get_time()
#         self.previous_state = None  # 前回の状態を追跡

#     def spin(self):
#         while not rospy.is_shutdown():
#             self.check_and_publish_state()
#             self.rate.sleep()

#     def check_and_publish_state(self):
#         global INTERACTING_FLAG
#         global TIME_CONTROLLER_LIST
#         global LAST_TIME
#         pub_msg_state = Int16()
#         pub_msg_demo_state = Bool()
        
#         # 状態を変更するためのフラグ
#         state_changed = False  

#         if CURRENT_UMORU_STATE in [1, 2, 3]:
#             if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 10:
#                 pub_msg_state.data = CURRENT_UMORU_STATE + 1
#                 state_changed = True
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
                
#         elif CURRENT_UMORU_STATE == 4:
#             if rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
#                 pub_msg_state.data = 5
#                 state_changed = True
#                 TIME_CONTROLLER_LIST.append(rospy.get_time())
                
#         elif CURRENT_UMORU_STATE == 5:
#             if len(TIME_CONTROLLER_LIST) == 0 or rospy.get_time() - TIME_CONTROLLER_LIST[-1] >= 20:
#                 pub_msg_state.data = 0
#                 pub_msg_demo_state.data = False
#                 state_changed = True
#                 TIME_CONTROLLER_LIST = []
#                 INTERACTING_FLAG = False
#                 LAST_TIME = rospy.get_time()
        
#         # 状態が変わり、前回の状態と異なる場合のみpublishする
#         if state_changed and pub_msg_state.data != self.previous_state:
#             self.previous_state = pub_msg_state.data  # 状態を更新
#             self.publish_state(pub_msg_state)
#             print(f"State updated to {pub_msg_state.data}")
        
#     def publish_state(self, data):
#         self.pub.publish(data)


class umoruStateController():
    def __init__(self):
        self.pub_heart_color = rospy.Publisher("/heart_color", Float32MultiArray, queue_size=10)
        self.pub_heart_pulse = rospy.Publisher("/pulse_time", Float32, queue_size=10)
        self.pub_eye_status = rospy.Publisher("/eye_status", UInt16, queue_size=10)
        self.pub_demo_status = rospy.Publisher("/demo_status", Bool, queue_size=10)
        self.pub_current_umoru_state = rospy.Publisher("/current_umoru_state", Int16, queue_size=10)
        # self.sub = rospy.Subscriber("/umoru_state", Int16, self.callback, queue_size=10)
        self.sub_change_trigger = rospy.Subscriber("/change_trigger", String, self.callback_change_trigger, queue_size=10)
        print("init umoru state controller")
        rospy.sleep(0.1)
        
    def play_sound_async(self, file_path):
        threading.Thread(target=playsound, args=(file_path,)).start()
    
    def execute_in_thread(self, function, *args, **kwargs):
        """
        任意の関数を別スレッドで実行するヘルパー関数
        """
        thread = threading.Thread(target=function, args=args, kwargs=kwargs)
        thread.start()

    def callback_change_trigger(self, data):
        pub_msg_heart_pulse = Float32()
        pub_msg_heart_color = Float32MultiArray()
        pub_msg_eye_status = UInt16()
        pub_msg_demo_status = Bool()
        
        global CURRENT_UMORU_STATE
        global TIME_CONTROLLER_LIST
        global USE_ARM
        global LAST_TIME
        global INTERACTING_FLAG

        change_trigger = data.data
        state_changed_flag = False

        if change_trigger == "time":
            state_changed_flag = True
            if CURRENT_UMORU_STATE == 5:
                CURRENT_UMORU_STATE = 0
            else:
                CURRENT_UMORU_STATE += 1
        elif change_trigger == "inflatable" or change_trigger == "voice":
            if 1 <= CURRENT_UMORU_STATE <= 3 and rospy.get_time() - TIME_CONTROLLER_LIST[CURRENT_UMORU_STATE - 1] > 10:
                CURRENT_UMORU_STATE += 1
                state_changed_flag = True
        elif change_trigger == "face":
            CURRENT_UMORU_STATE = 1
            state_changed_flag = True

        if state_changed_flag:
            print(f"!!!!!!!!!!!!!changed umoru state by {change_trigger}!!!!!!!!!!!!!!!!!!!!!!!!!")
            if CURRENT_UMORU_STATE == 0:
                pub_msg_heart_pulse.data = 10
                pub_msg_heart_color.data = [0.01, 0.01, 0.01]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST = []
                INTERACTING_FLAG = False
                LAST_TIME = rospy.get_time()
                print("============== state = 0 (interaction was reset)  ===================")
            elif CURRENT_UMORU_STATE == 1:
                pub_msg_heart_pulse.data = 2.0
                pub_msg_heart_color.data = [0.9,0.9,0.9]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                self.play_sound_async("/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-first-phrase.wav")
                print("=============== state = 1 (heart appeared) ========================")
            elif CURRENT_UMORU_STATE == 2:
                pub_msg_heart_pulse.data = 1
                pub_msg_heart_color.data = [0.9,0.5,0.8]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-yobimashitaka.wav",
                                                     "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-bikkuri.wav",
                                                     "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-iinioi.wav"]))
                print("=============== state = 2 (heart turned pink) ========================")
            elif CURRENT_UMORU_STATE == 3:
                pub_msg_heart_pulse.data = 0.8
                pub_msg_heart_color.data = [0.9,0.3,0.5]
                pub_msg_eye_status.data = 1
                pub_msg_demo_status.data = 1
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-nukumori.wav",
                                                     "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-kodou.wav",
                                                     "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-kokyu-fukkatsu.wav"]))
                print("=============== state = 3 (breathing start)  ========================")
            elif CURRENT_UMORU_STATE == 4:
                pub_msg_heart_pulse.data = 0.6
                pub_msg_heart_color.data = [0.9,0,0]
                pub_msg_demo_status.data = 1
                pub_msg_eye_status.data = 1
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                self.play_sound_async("/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-hug-fukkatsu.wav")
                if USE_ARM == True:
                    # arm_client.reset_pose()
                    # arm_client.hug()
                    self.execute_in_thread(arm_client.reset_pose())
                    self.execute_in_thread(arm_client.hug())
                print("=============== state = 4 (hug start) ========================")
            elif CURRENT_UMORU_STATE == 5:
                pub_msg_heart_pulse.data = 0.4
                pub_msg_heart_color.data = [0.9,0,0]
                pub_msg_demo_status.data = 1
                pub_msg_eye_status.data = 1
                self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-thankyou-pattern1.wav",
                                                     "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-thankyou-pattern2.wav"]))
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                if USE_ARM == True:
                    # arm_client.reset_pose()
                    # arm_client.init_pose()
                    self.execute_in_thread(arm_client.reset_pose())
                    self.execute_in_thread(arm_client.init_pose())
                print("=============== state = 5 (stop hugging) ========================")

            self.publish_heart_pulse(pub_msg_heart_pulse)
            self.publish_heart_color(pub_msg_heart_color)
            self.publish_demo_status(pub_msg_demo_status)
            self.publish_eye_status(pub_msg_eye_status)
            # if pub_msg_heart_pulse.data != 0:
            #     self.publish_heart_pulse(pub_msg_heart_pulse)
            #     time.sleep(0.1)
            # pub_msg_current_umoru_state = Int16()
            # pub_msg_current_umoru_state.data = CURRENT_UMORU_STATE
            # self.pub_current_umoru_state.publish(pub_msg_current_umoru_state)
        rospy.sleep(0.05)
        
    def callback(self, data):
        pub_msg_heart_pulse = Float32()
        pub_msg_heart_color = Float32MultiArray()
        pub_msg_eye_status = UInt16()
        pub_msg_demo_status = Bool()
        
        global CURRENT_UMORU_STATE
        global TIME_CONTROLLER_LIST
        global USE_ARM
        print(f"[callback_before]current_umoru_state:{CURRENT_UMORU_STATE}, data:{data.data}")
        if CURRENT_UMORU_STATE < int(data.data) or (CURRENT_UMORU_STATE == 5 and int(data.data) == 0):
            if (CURRENT_UMORU_STATE == 5 and data.data == 0):
                CURRENT_UMORU_STATE = data.data
            # elif CURRENT_UMORU_STATE == 4:
                # CURRENT_UMORU_STATE = 5
            else:
                CURRENT_UMORU_STATE = CURRENT_UMORU_STATE + 1
            print(f"[callback_after]current_umoru_state:{CURRENT_UMORU_STATE}, data:{data.data}")
            if CURRENT_UMORU_STATE == 0:
                pub_msg_heart_pulse.data = 10
                pub_msg_heart_color.data = [0.01, 0.01, 0.01]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                print("============== state = 0 (interaction was reset)  ===================")
            elif CURRENT_UMORU_STATE == 1:
                CURRENT_UMORU_STATE = 1
                pub_msg_heart_pulse.data = 2.0
                pub_msg_heart_color.data = [0.9,0.9,0.9]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                self.play_sound_async("/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-first-phrase.wav")
                time_to_sleep = 10
                print("=============== state = 1 (heart appeared) ========================")
            elif CURRENT_UMORU_STATE == 2 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]:
                CURRENT_UMORU_STATE = 2
                pub_msg_heart_pulse.data = 1
                pub_msg_heart_color.data = [0.9,0.5,0.8]
                pub_msg_eye_status.data = 3
                pub_msg_demo_status.data = 0
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-yobimashitaka.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-bikkuri.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-iinioi.wav"]))
                print("=============== state = 2 ========================")
            elif (CURRENT_UMORU_STATE == 3 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]):
                CURRENT_UMORU_STATE = 3
                pub_msg_heart_pulse.data = 0.8
                pub_msg_heart_color.data = [0.9,0.3,0.5]
                pub_msg_eye_status.data = 1
                pub_msg_demo_status.data = 1
                self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-nukumori.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-kodou.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-kokyu-fukkatsu.wav"]))
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                print("=============== state = 3 ========================")
                print("=============== 呼吸スタート =====================")
            elif (CURRENT_UMORU_STATE == 4 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]):
                self.play_sound_async("/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-hug-fukkatsu.wav")
                if USE_ARM == True:
                    # arm_client.reset_pose()
                    # arm_client.hug()
                    self.execute_in_thread(arm_client.reset_pose())
                    self.execute_in_thread(arm_client.hug())
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                CURRENT_UMORU_STATE = 4
                pub_msg_heart_pulse.data = 0.6
                pub_msg_heart_color.data = [0.9,0,0]
                pub_msg_demo_status.data = 1
                pub_msg_eye_status.data = 1
                print("=============== state = 4 ========================")
                print("=============== ハグスタート ======================")
            elif CURRENT_UMORU_STATE == 5 and 10 < rospy.get_time() - TIME_CONTROLLER_LIST[-1]:
                CURRENT_UMORU_STATE = 5
                print("ここにはいってほしいーーーーー")
                self.play_sound_async(random.choice(["/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-thankyou-pattern1.wav", "/home/leus/seisakuten_ws/src/umoru_main/src/sounds/umoru-thankyou-pattern2.wav"]))
                if USE_ARM == True:
                    # arm_client.reset_pose()
                    # arm_client.init_pose()
                    self.execute_in_thread(arm_client.reset_pose())
                    self.execute_in_thread(arm_client.init_pose())
                TIME_CONTROLLER_LIST.append(rospy.get_time())
                pub_msg_eye_status.data = 1
                pub_msg_demo_status.data = 1
                pub_msg_heart_pulse.data = 0.4
                pub_msg_heart_color.data = [0.9,0,0]
                
                print("=============== state = 5 ========================")
                print("=============== ハグ終了 ======================")

            self.publish_demo_status(pub_msg_demo_status)
            if pub_msg_heart_pulse.data != 0:
                self.publish_heart_pulse(pub_msg_heart_pulse)
                time.sleep(0.1)
            self.publish_heart_color(pub_msg_heart_color)
            self.publish_eye_status(pub_msg_eye_status)
            pub_msg_current_umoru_state = Int16()
            pub_msg_current_umoru_state.data = CURRENT_UMORU_STATE
            self.pub_current_umoru_state.publish(pub_msg_current_umoru_state)
        rospy.sleep(0.05)
        
    def publish_heart_pulse(self, data):
        self.pub_heart_pulse.publish(data)

    def publish_heart_color(self, data):
        self.pub_heart_color.publish(data)
    
    def publish_eye_status(self, data):
        self.pub_eye_status.publish(data)

    def publish_demo_status(self, data):
        self.pub_demo_status.publish(data)


if __name__ == '__main__':
    rospy.init_node("umoru_main")
    if USE_ARM == True:
        arm_client = MotionClient("both")
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
