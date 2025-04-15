# speech_to_text.py

from faster_whisper import WhisperModel
import config
import rclpy

class SpeechToText:
    def __init__(self):
        self.model = WhisperModel(config.WHISPER_MODEL, device=config.DEVICE, compute_type=config.COMPUTE_TYPE)

    def transcribe(self, audio_path):

        segments, info = self.model.transcribe(audio_path, language=None)
        text = "".join([seg.text for seg in segments])
        detected_lang = info.language
        print(f"📝 Whisper识别内容: {text} | 检测语言: {detected_lang}")
        return text, detected_lang
