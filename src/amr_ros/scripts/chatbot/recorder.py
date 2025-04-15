# recorder.py

import sounddevice as sd
import numpy as np
import scipy.io.wavfile
import config
import time
import rclpy

def record_audio(filename="input.wav", fs=config.SAMPLE_RATE,
                 max_duration=10,
                 silence_threshold=100,
                 silence_duration=1.5):

    print("🎙️ 开始录音...")

    buffer = []
    silence_start = None

    def callback(indata, frames, time_info, status):
        nonlocal silence_start
        volume_norm = np.linalg.norm(indata) * 1000
        print(f"音量: {volume_norm:.2f}")
        buffer.append(indata.copy())

        if volume_norm < silence_threshold:
            if silence_start is None:
                silence_start = time.time()
            elif time.time() - silence_start > silence_duration:
                raise sd.CallbackStop()
        else:
            silence_start = None

    try:
        with sd.InputStream(channels=1, samplerate=fs, callback=callback):
            sd.sleep(int(max_duration * 1000))
    except sd.CallbackStop:
        pass

    if not buffer:
        print("⚠️ 没有录到声音")
        return None

    audio = np.concatenate(buffer, axis=0)
    audio_int16 = np.int16(audio * 32767)

    scipy.io.wavfile.write(filename, fs, audio_int16)
    print(f"✅ 录音完成，保存至 {filename}")
    return filename
