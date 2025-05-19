import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
import time

SAMPLE_RATE = 16000  # 采样率
DURATION_BG = 5      # 背景噪声采样时间（秒）
THRESHOLD_MULTIPLIER = 13000  # 阈值放大倍数 1.3倍

def get_rms(data):
    """计算均方根音量"""
    return np.sqrt(np.mean(np.square(data)))

def record_audio(duration, sample_rate):
    """录制音频并返回numpy数组"""
    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float32')
    sd.wait()
    return audio

print("📡 正在采样背景噪声 {} 秒...".format(DURATION_BG))
bg_data = record_audio(DURATION_BG, SAMPLE_RATE)
bg_rms = get_rms(bg_data)
threshold = bg_rms * THRESHOLD_MULTIPLIER
print(f"✅ 背景噪声阈值设置为: {threshold:.2f}")

# 保存阈值到本地文件
with open("threshold.txt", "w") as f:
    f.write(str(threshold))
