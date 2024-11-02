# import numpy as np
# import rospy
# from audio_common_msgs.msg import AudioData
# import math
# from collections import deque

# # パラメータ設定
# BUFFER_DURATION = 2.0  # バッファの長さ（秒単位）
# SAMPLE_RATE = 16000  # サンプリングレート（Hz）
# BUFFER_SIZE = int(SAMPLE_RATE * BUFFER_DURATION)  # バッファのサイズ

# audio_buffer = deque(maxlen=BUFFER_SIZE)

# def audio_callback(msg):
#     global audio_buffer
#     audio_data = np.frombuffer(msg.data, dtype=np.int16)
    
#     # 音声データをバッファに追加
#     audio_buffer.extend(audio_data)

#     # 一定時間分のデータが揃ったら平均値と対数を計算
#     if len(audio_buffer) == BUFFER_SIZE:
#         # 音量のRMS計算
#         buffer_data = np.array(audio_buffer)
#         mean_square = np.mean(buffer_data ** 2)
        
#         # 平均音量がゼロの場合は処理
#         if mean_square == 0 or math.isnan(mean_square):
#             volume = 0.0
#         else:
#             rms_volume = np.sqrt(mean_square)
#             volume = 0.0 if rms_volume <= 0 else math.log10(rms_volume)

#         rospy.loginfo(f"Average Log Volume: {volume}")

# rospy.init_node('average_log_volume_calculator')
# rospy.Subscriber('/audio', AudioData, audio_callback)
# rospy.spin()

import numpy as np
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Float32
import math

rospy.init_node('log_volume_publisher')
pub = rospy.Publisher('/log_volume', Float32, queue_size=10)

SAMPLE_RATE = 16000  # サンプリングレート（Hz）
WINDOW_DURATION = 1.0  # ウィンドウの長さ（秒）
WINDOW_SIZE = int(SAMPLE_RATE * WINDOW_DURATION)  # ウィンドウのサイズ

def audio_callback(msg):
    audio_data = np.frombuffer(msg.data, dtype=np.int16)

    for i in range(0, len(audio_data), WINDOW_SIZE):
        window_data = audio_data[i:i + WINDOW_SIZE]
        if len(window_data) > 0:
            mean_square = np.mean(window_data ** 2)
            if mean_square <= 0 or math.isnan(mean_square):
                log_volume = 0.0
            else:
                rms_volume = np.sqrt(mean_square)
                log_volume = 0.0 if rms_volume <= 0 else math.log10(rms_volume)

            pub.publish(log_volume)

rospy.Subscriber('/audio', AudioData, audio_callback)
rospy.spin()

