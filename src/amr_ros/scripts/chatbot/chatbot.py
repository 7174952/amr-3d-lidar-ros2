import asyncio
import rclpy
from chatbot_node import ChatBotNode
import config
from hotword_detector import HotwordDetector
from recorder import record_audio
from speech_to_text import SpeechToText
from chat_gpt import ChatGPT
from text_to_speech import TextToSpeech
import scipy.io.wavfile
import sounddevice as sd
from pydub import AudioSegment
from pydub.playback import play
import time

def play_ding(filePath):
    fs, data = scipy.io.wavfile.read(filePath)
    sd.play(data, fs)
    sd.wait()

def play_silence(duration_sec=0.2):
    silence = AudioSegment.silent(duration=duration_sec * 1000)
    play(silence)

async def chatbot_logic(node: ChatBotNode):
    hotword_detector = HotwordDetector(node)
    stt = SpeechToText()
    chat_gpt = ChatGPT()
    tts = TextToSpeech()

    print("Enter Sleep Mode!")
    curr_state = "state:sleep;"
    node.publish_text(curr_state)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        if(not node.req_wakeup): #sleep
            continue
        else: #wakeup
            if curr_state == "state:sleep;":
                curr_state = "state:ready;"
                node.publish_text(curr_state)
            result = hotword_detector.detect()
            if result == -1:
                print("Exit")
                break
            elif result == 0:
                print("Go to Sleep Mode")
                curr_state = "state:sleep;"
                node.publish_text(curr_state)
                continue
            else:
                curr_state = "state:wakeup;"
                node.publish_text(curr_state)
                # time.sleep(0.1)

            audio_path = record_audio()
            rclpy.spin_once(node,  timeout_sec=0.001)
            if not node.req_wakeup:
                curr_state = "state:sleep;"
                node.publish_text(curr_state)
                continue #goto sleep

            user_text, lang = stt.transcribe(audio_path)
            rclpy.spin_once(node,  timeout_sec=0.001)
            if not node.req_wakeup:
                curr_state = "state:sleep;"
                node.publish_text(curr_state)
                continue #goto sleep

            if user_text.strip() == "" or lang not in ["en", "ja", "zh"]:
                curr_state = "state:ready;"
                node.publish_text(curr_state)
                continue
            else:
                curr_state = "state:question;" + user_text
                node.publish_text(curr_state)

            node.get_logger().info(f"识别结果: {user_text}")
            # node.publish_text(user_text)

            if any(word in user_text for word in ["再见", "拜拜", "再見", "掰掰","bye","Bye","BYE", "バイバイ"]):
                play_silence()
                play_ding(config.EXIT_RESP_PATH)
                curr_state = "state:ready;"
                node.publish_text(curr_state)
                continue

            reply = chat_gpt.get_response(user_text, lang)
            if isinstance(reply, dict):
                reply_text = reply.get("result", "わかりませんでした。")
            else:
                reply_text = str(reply)
            # node.publish_text(reply_text)
            rclpy.spin_once(node,  timeout_sec=0.001)
            if not node.req_wakeup:
                curr_state = "state:sleep;"
                node.publish_text(curr_state)
                continue #goto sleep
            else:
                curr_state = "state:answer;" + reply_text
                node.publish_text(curr_state)

            tts.tts_with_openai(reply_text, lang)
            rclpy.spin_once(node,  timeout_sec=0.001)
            if not node.req_wakeup:
                curr_state = "state:sleep;"
                node.publish_text(curr_state)
                continue #goto sleep
            else:
                curr_state = "state:speech;"
                node.publish_text(curr_state)
                time.sleep(0.01)
                curr_state = "state:ready;"
                node.publish_text(curr_state)

    hotword_detector.close()

def main():
    rclpy.init()
    node = ChatBotNode()
    try:
        asyncio.run(chatbot_logic(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
