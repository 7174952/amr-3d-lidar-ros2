#!/usr/bin/env python3
from openai import OpenAI
import os
from playsound import playsound

client = OpenAI()
client.api_key = os.getenv('OPENAI_API_KEY')


speech_file_path = "hotword_resp" + ".wav"
content_text = "はい"

response = client.audio.speech.create(
    model="tts-1",
    voice="nova",
    input=content_text,
    response_format="wav"
)

response.stream_to_file(speech_file_path)

