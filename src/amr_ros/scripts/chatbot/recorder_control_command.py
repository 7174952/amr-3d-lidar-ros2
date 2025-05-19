# recorder_control_command.py

import sounddevice as sd
import numpy as np
import scipy.io.wavfile
import config
import time
import rclpy

def record_audio_control_command(filename="input.wav",
                 fs=config.SAMPLE_RATE,
                 max_duration=10,
                 silence_threshold=100,
                 silence_duration=1.0,
                 sleep_interval=0.1,     # 💤 无语音时定期让出控制权
                 check_duration=0.5,
                 ros_node=None):    # 每次检测音量的持续时间（秒）
    print(f"silence_threshold: {silence_threshold}")
    print("🎙️ 等待用户说话中...")

    buffer = []
    silence_start = None
    recording = False

    def rms_volume(data):
        return np.sqrt(np.mean(data**2)) * 1000

    start_time = time.time()

    try:
        with sd.InputStream(channels=1, samplerate=fs) as stream:
            while True:
                # 读入音频数据
                audio_chunk, _ = stream.read(int(fs * check_duration))
                volume = rms_volume(audio_chunk)
                print(f"音量: {volume:.2f}")

                # ✅ 未开始录音：检测是否有人说话
                if not recording:
                    if volume >= silence_threshold:
                        print("🎤 检测到语音，开始录音...")
                        recording = True
                        buffer.append(audio_chunk.copy())
                        silence_start = None
                    else:
                        time.sleep(sleep_interval)  # 🔁 让出控制权供 ROS2 处理
                        if ros_node is not None:
                            rclpy.spin_once(ros_node, timeout_sec=0.001)
                        continue

                # ✅ 已在录音中
                else:
                    buffer.append(audio_chunk.copy())

                    if volume < silence_threshold:
                        if silence_start is None:
                            silence_start = time.time()
                        elif time.time() - silence_start > silence_duration:
                            print("🛑 静音超时，停止录音")
                            break
                    else:
                        silence_start = None

                # ⏱️ 最大录音时长保护
                if time.time() - start_time > max_duration:
                    print("⏱️ 达到最大录音时长")
                    break

    except Exception as e:
        print(f"❌ 录音发生错误: {e}")
        return None

    if not buffer:
        print("⚠️ 没有录到声音")
        return None

    audio = np.concatenate(buffer, axis=0)
    audio_int16 = np.int16(audio * 32767)
    scipy.io.wavfile.write(filename, fs, audio_int16)
    print(f"✅ 录音完成，保存至 {filename}")
    return filename
