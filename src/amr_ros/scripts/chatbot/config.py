import os

KEYWORD_PATH = [
    # "keywords/ni-hao-zhu-shou_zh.ppn",
    "keywords/りんご_linux.ppn",
    "keywords/mikurobo_ja.ppn"
]
API_KEY = os.getenv("OPENAI_API_KEY")
RECORD_DURATION = 5  # 秒
SAMPLE_RATE = 16000
MODEL_PATH = "keywords/porcupine_params_ja.pv"
WAKEUP_ACCESS_KEY = os.getenv("WAKEUP_ACCESS_KEY")
WHISPER_MODEL = "base"
DEVICE = "cpu"
COMPUTE_TYPE = "int8"

TEXT_FILE_PATH = {
    "ja":"assets/brief_ja.txt",
    "en":"assets/brief_en.txt",
    "zh":"assets/brief_zh.txt"
}
TEXT_FILE_SEPERATOR = {
    "ja":"。",
    "en":".",
    "zh":"。"
}
HOTWORD_RESP_PATH = "assets/hotword_resp.wav"
SPEECH_FILE_PATH = "assets/speech.mp3"
EXIT_RESP_PATH = "assets/bye_bye.wav"
