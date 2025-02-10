# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from examples.drive_controller import RobotController
from elevenlabs.client import ElevenLabs
import sounddevice as sd
import soundfile as sf
from scipy import signal
from io import BytesIO
import dotenv
import alsaaudio
import threading
import queue
import numpy as np
from pi5neo import Pi5Neo

# LED and Audio Constants
CHUNK_DURATION = 0.1
MAX_LEDS = 15
LED_COLOR = (0, 0, 128)
DEVICE_NAME = "UACDemoV1.0"
LED_DEVICE_PATH = "/dev/spidev0.0"
LED_BAUDRATE = 800
VOLUME_SENSITIVITY = 10
RAMP_SPEED = 1

class TTSWithLED:
    def __init__(self):
        dotenv.load_dotenv()
        self.set_alsa_volume()
        
    def set_alsa_volume(self, volume=80):
        try:
            cards = alsaaudio.cards()
            card_num = None
            for i, card in enumerate(cards):
                if 'UACDemoV10' in card:
                    card_num = i
                    break
            
            if card_num is None:
                print("Could not find UACDemoV1.0 audio device")
                return
                
            mixer = alsaaudio.Mixer('PCM', cardindex=card_num)
            mixer.setvolume(volume)
        except alsaaudio.ALSAAudioError as e:
            print(f"Error setting volume: {e}")

    def play_text_with_led(self, text):
        api_key = os.getenv("ELEVENLABS_API_KEY")
        client = ElevenLabs(api_key=api_key)

        audio = client.generate(
            text=text,
            voice="ceicSWVDzgXoWidth8WQ", #raphael
            model="eleven_multilingual_v2"
        )

        device_info = sd.query_devices(DEVICE_NAME, 'output')
        device_id = device_info['index']
        device_sample_rate = int(device_info['default_samplerate'])

        audio_data = b''.join(audio)
        data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

        if sample_rate != device_sample_rate:
            number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
            data = signal.resample(data, number_of_samples)
            sample_rate = device_sample_rate

        chunk_samples = int(sample_rate * CHUNK_DURATION)
        volume_queue = queue.Queue(maxsize=1)
        index = 0
        stop_event = threading.Event()

        def audio_callback(outdata, frames, time_info, status):
            nonlocal index
            if status:
                print(f"Audio Callback Status: {status}")

            end_index = index + frames
            if end_index > len(data):
                out_frames = len(data) - index
                outdata[:out_frames, 0] = data[index:index + out_frames]
                outdata[out_frames:, 0] = 0
                index += out_frames
                raise sd.CallbackStop()
            else:
                outdata[:, 0] = data[index:end_index]
                index += frames

            current_chunk = data[max(0, index - chunk_samples):index]
            avg_volume = np.sqrt(np.mean(current_chunk**2)) if len(current_chunk) > 0 else 0

            try:
                if volume_queue.full():
                    volume_queue.get_nowait()
                volume_queue.put_nowait(avg_volume)
            except queue.Full:
                pass

        def led_update_thread():
            neo = Pi5Neo(LED_DEVICE_PATH, MAX_LEDS, LED_BAUDRATE)
            current_led_count = 0.0

            while not stop_event.is_set():
                try:
                    avg_volume = volume_queue.get(timeout=CHUNK_DURATION)
                except queue.Empty:
                    avg_volume = 0

                desired_led_count = min(MAX_LEDS, avg_volume * MAX_LEDS * VOLUME_SENSITIVITY)

                if current_led_count < desired_led_count:
                    current_led_count = min(current_led_count + RAMP_SPEED, desired_led_count)
                elif current_led_count > desired_led_count:
                    current_led_count = max(current_led_count - RAMP_SPEED, desired_led_count)

                neo.clear_strip()
                for j in range(int(current_led_count)):
                    neo.set_led_color(j, *LED_COLOR)
                neo.update_strip()

            neo.clear_strip()
            neo.update_strip()

        led_thread = threading.Thread(target=led_update_thread)
        led_thread.start()

        try:
            with sd.OutputStream(
                device=device_id,
                samplerate=sample_rate,
                channels=1,
                callback=audio_callback,
                dtype='float32',
                latency='low',
            ):
                sd.sleep(int(len(data) / sample_rate * 1000))
        finally:
            stop_event.set()
            led_thread.join()

def main():
    try:
        # Initialize the robot
        robot = RobotController()

        robot.drive_distance(1.0)
        robot.turn_degrees(90)
        
        # Initialize TTS with LED and play announcement
        tts_led = TTSWithLED()
        tts_led.play_text_with_led("Officially announcing the bracket bot hackathon")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.cleanup()

if __name__ == "__main__":
    main()