import rospy
import time
# import module
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import json
from PIL import Image
from io import BytesIO
import numpy as np
import MeCab
from wordcloud import WordCloud, ImageColorGenerator
import matplotlib.pyplot as plt
# from openai import OpenAI
import re
import datetime
import requests
from sound_play.libsoundplay import SoundClient
# from umoru_arm import MotionClient
from audio_common_msgs.msg import AudioData
import math
from collections import deque

# global hug_point
# global hug_point_diff_by_pressure
# global hug_point_diff_by_voice

global CURRENT_UMORU_STATE
CURRENT_UMORU_STATE = 0

class pressureSub():
    """
    気圧センサによって加えるポイントを決定する
    """
    def __init__(self):
        # Subscriberの作成
        self.sub = rospy.Subscriber("/air_pressure", Float32, self.callback)
        time.sleep(1)

    def callback(self, data):
        hug_point_diff_by_pressure = data.data * 0.001

class voiceSub():
    def __init__(self):
        # パラメータ設定
        BUFFER_DURATION = 1.0  # バッファの長さ（秒単位）
        SAMPLE_RATE = 16000  # サンプリングレート（Hz）
        self.BUFFER_SIZE = int(SAMPLE_RATE * BUFFER_DURATION)  # バッファのサイズ
        self.audio_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.sub = rospy.Subscriber('/audio', AudioData, self.callback)
        self.pub_state = rospy.Publisher('/umoru_state', Int16, queue_size=1)
        self.pub_volume = rospy.Publisher('/log_volume', Float32, queue_size=10)
        
    def callback(self, data):
        pub_msg_state = Int16()
        pub_msg_volume = Float32()
        audio_data = np.frombuffer(data.data, dtype=np.int16)
        
        # 音声データをバッファに追加
        self.audio_buffer.extend(audio_data)
        rospy.sleep(0.1)

        # 一定時間分のデータが揃ったら平均値と対数を計算
        if len(self.audio_buffer) == self.BUFFER_SIZE:
            # 音量のRMS計算
            buffer_data = np.array(self.audio_buffer)
            mean_square = np.mean(buffer_data ** 2)
            
            # 平均音量がゼロの場合は処理
            if mean_square == 0 or math.isnan(mean_square):
                volume = 0.0
            else:
                rms_volume = np.sqrt(mean_square)
                volume = 0.0 if rms_volume <= 0 or math.isnan(rms_volume) else math.log10(rms_volume)
            
            # hug_point_diff_by_voice = volume * 10
            rospy.loginfo(f"Average Log Volume: {volume}")
            
            pub_msg_volume.data = volume
            self.publish_volume(pub_msg_volume)

            if volume < 0.5:
                pub_msg_state.data = 0
            elif volume >= 0.5 and volume < 1.0:
                pub_msg_state.data = 1
            elif volume >= 1.0 and volume < 1.5:
                pub_msg_state.data = 2
            elif volume >= 1.5:
                pub_msg_state.data = 3
            self.publish_state(pub_msg_state)
            
    def publish_state(self, data):
        self.pub_state.publish(data)
    
    def publish_volume(self, data):
        self.pub_volume.publish(data)
    

class umoruStateController():
    def __init__(self):
        self.pub_heart_color = rospy.Publisher("/heart_color", Float32MultiArray, queue_size=1)
        self.pub_heart_pulse = rospy.Publisher("/pulse_time", Float32, queue_size=1)
        self.sub = self.sub = rospy.Subscriber("/umoru_state", Int16, self.callback)
    
    def callback(self, data):
        pub_msg_heart_pulse = Float32()
        pub_msg_heart_color = Float32MultiArray()
        global CURRENT_UMORU_STATE

        if CURRENT_UMORU_STATE < int(data.data):
            CURRENT_UMORU_STATE = int(data.data)
        
        if CURRENT_UMORU_STATE == 0:
            pub_msg_heart_pulse.data = 2.0
            pub_msg_heart_color.data = [1.0,0.9,0.9]
        
        elif CURRENT_UMORU_STATE == 1:
            pub_msg_heart_pulse.data = 1.0
            pub_msg_heart_color.data = [1.0,0.5,0.8]

        elif CURRENT_UMORU_STATE == 2:
            pub_msg_heart_pulse.data = 0.5
            pub_msg_heart_color.data = [1.0,0.3,0.5]
        
        elif CURRENT_UMORU_STATE == 3:
            pub_msg_heart_pulse.data = 0.2
            pub_msg_heart_color.data = [1.0,0,0]
        
        self.publish_heart_pulse(pub_msg_heart_pulse)
        self.publish_heart_color(pub_msg_heart_color)

    def publish_heart_pulse(self, data):
        self.pub_heart_pulse.publish(data)
    
    def publish_heart_color(self, data):
        self.pub_heart_color.publish(data)

if __name__ == '__main__':
    rospy.init_node("test_node")
    node1 = voiceSub()
    node2 = umoruStateController()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)