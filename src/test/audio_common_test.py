import rospy
import numpy as np
from audio_common_msgs.msg import AudioData

def audio_callback(msg):
    # バイナリデータをnumpy配列に変換
    audio_data = np.frombuffer(msg.data, dtype=np.int16)
    
    # RMS（音量）を計算
    rms = np.sqrt(np.mean(np.square(audio_data)))
    
    # デシベル (dB) に変換 (オプション)
    if rms > 0:
        db = 20 * np.log10(rms)
        rospy.loginfo(f"RMS: {rms:.2f}, Volume (dB): {db:.2f}")
    else:
        rospy.loginfo("Silence detected")

def listener():
    rospy.init_node('audio_volume_calculator')
    rospy.Subscriber("/audio", AudioData, audio_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
