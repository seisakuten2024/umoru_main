import numpy as np
import rospy
from audio_common_msgs.msg import AudioData
import math
from collections import deque

# パラメータ設定
BUFFER_DURATION = 2.0  # バッファの長さ（秒単位）
SAMPLE_RATE = 16000  # サンプリングレート（Hz）
BUFFER_SIZE = int(SAMPLE_RATE * BUFFER_DURATION)  # バッファのサイズ

audio_buffer = deque(maxlen=BUFFER_SIZE)

def audio_callback(msg):
    global audio_buffer
    audio_data = np.frombuffer(msg.data, dtype=np.int16)
    
    # 音声データをバッファに追加
    audio_buffer.extend(audio_data)

    # 一定時間分のデータが揃ったら平均値と対数を計算
    if len(audio_buffer) == BUFFER_SIZE:
        # 音量のRMS計算
        buffer_data = np.array(audio_buffer)
        mean_square = np.mean(buffer_data ** 2)
        
        # 平均音量がゼロの場合は処理
        if mean_square == 0 or math.isnan(mean_square):
            volume = 0.0
        else:
            rms_volume = np.sqrt(mean_square)
            volume = 0.0 if rms_volume <= 0 else math.log10(rms_volume)

        rospy.loginfo(f"Average Log Volume: {volume}")

rospy.init_node('average_log_volume_calculator')
rospy.Subscriber('/audio', AudioData, audio_callback)
rospy.spin()
