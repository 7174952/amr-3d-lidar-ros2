# text_to_speech.py

from pathlib import Path
from openai import OpenAI
import config

class TextToSpeech:
    def __init__(self):
        self.openai = OpenAI(api_key=config.API_KEY)

    def tts_with_openai(self, text, lang_code="ja"):
        voice = {"zh": "nova", "en": "nova", "ja": "nova"}.get(lang_code, "nova")
        with self.openai.audio.speech.with_streaming_response.create(
            model="gpt-4o-mini-tts",
            voice=voice,
            input=text,
            instructions="Speak in a cheerful and positive tone.",
        ) as response:
            response.stream_to_file(config.SPEECH_FILE_PATH)

