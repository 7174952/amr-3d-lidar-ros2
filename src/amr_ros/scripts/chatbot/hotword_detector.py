# hotword_detector.py

import pvporcupine
import pyaudio
import struct
import rclpy
import config
import os
import asyncio
import scipy.io.wavfile
import sounddevice as sd
from pydub import AudioSegment
from pydub.playback import play
import time

class HotwordDetector:
    def __init__(self,node):
        self.node = node
        self.porcupine = pvporcupine.create(
            access_key=config.WAKEUP_ACCESS_KEY,
            keyword_paths=config.KEYWORD_PATH,
            model_path=config.MODEL_PATH
        )
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length,
        )

    def _play_silence(self, duration_sec=0.2):
        silence = AudioSegment.silent(duration=duration_sec * 1000)  # pydub是毫秒单位
        play(silence)

    def _play_ding(self):
        self._play_silence()
        fs, data = scipy.io.wavfile.read(config.HOTWORD_RESP_PATH)
        sd.play(data, fs)
        sd.wait()
        print("🔔 响应音播放完毕")

    def detect(self):
        print("🎤 说出唤醒词（ミクロボ）...")
        try:
            while self.node.context.ok():
                if not self.node.req_wakeup:
                    return 0
                # 如果缓存区不够，不 read，sleep 一下
                if self.stream.get_read_available() >= self.porcupine.frame_length:
                    pcm = self.stream.read(
                        self.porcupine.frame_length,
                        exception_on_overflow=False
                    )
                    pcm = struct.unpack_from(
                        "h" * self.porcupine.frame_length, pcm
                    )
                    result = self.porcupine.process(pcm)
                    if result >= 0:
                        print("✅ 唤醒词触发！")
                        self._play_ding()
                        return 1
                else:
                    rclpy.spin_once(self.node,  timeout_sec=0.01)

        except KeyboardInterrupt:
            print("检测被中断，退出 detect()")

        return -1

    def close(self):
        if self.stream.is_active():
            self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()
        self.porcupine.delete()
