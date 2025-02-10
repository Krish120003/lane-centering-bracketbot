# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from lib.camera import RealsenseCamera, USBCamera  # Using the new camera.py library


import time
import shutil
import boto3
import cv2
from io import BytesIO
import sounddevice as sd
import soundfile as sf
from scipy import signal
import dotenv
from botocore.exceptions import ClientError
from elevenlabs import ElevenLabs
import pyaudio
import wave
import openai
from tempfile import NamedTemporaryFile
import alsaaudio

dotenv.load_dotenv()

def set_alsa_volume(volume=100):
    try:
        # Find the card number for UACDemoV1.0
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
        print(f"Set UACDemoV1.0 volume to {volume}%")
    except alsaaudio.ALSAAudioError as e:
        print(f"Error setting volume: {e}")

def play_text_sound(text):
    api_key = os.getenv("ELEVENLABS_API_KEY")
    client = ElevenLabs(api_key=api_key)

    audio = client.generate(
        text=text,
        voice="Brian",
        # voice="0m2tDjDewtOfXrhxqgrJ",
        model="eleven_multilingual_v2"
    )

    device_name = "UACDemoV1.0"
    device_info = sd.query_devices(device_name, 'output')
    device_id = device_info['index']
    device_sample_rate = device_info['default_samplerate']

    audio_data = b''.join(audio)
    data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

    if sample_rate != device_sample_rate:
        number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
        data = signal.resample(data, number_of_samples)
        sample_rate = device_sample_rate

    data = data * 1.0

    try:
        sd.play(data, samplerate=sample_rate, device=device_id)
        sd.wait()
    except sd.PortAudioError as e:
        print(f"Error playing audio: {e}")

def record_name(duration=3):
    CHUNK = 2048
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    
    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )
    
    print(f"Recording for {duration} seconds...")
    frames = []
    
    for _ in range(0, int(RATE / CHUNK * duration)):
        data = stream.read(CHUNK)
        frames.append(data)
    
    print("Recording finished!")
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    with NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wf = wave.open(f.name, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        return f.name

def transcribe_audio(audio_file):
    client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    
    with open(audio_file, "rb") as audio:
        transcript = client.audio.transcriptions.create(
            model="whisper-1",
            file=audio
        )
    return transcript.text

def extract_name_from_transcript(transcript):
    client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": "You are a helpful assistant that extracts names from transcripts. You only respond with the name."},
            {"role": "user", "content": "Extract the name from the following transcript: " + transcript}
        ]
    )
    return response.choices[0].message.content

def search_faces(client, collection_id, image_path, last_name):
    with open(image_path, 'rb') as image:
        try:
            response = client.search_faces_by_image(
                CollectionId=collection_id,
                Image={'Bytes': image.read()},
                MaxFaces=1,
                FaceMatchThreshold=95
            )
            
            if response['FaceMatches']:
                match = response['FaceMatches'][0]
                name = match['Face']['ExternalImageId']
                
                if name != last_name:
                    play_text_sound(f"Hi {name}")
                
                return match, name
            else:
                if last_name is None:
                    play_text_sound("I don't know you. What is your name?")
                    
                    audio_file = record_name(3)
                    transcript = transcribe_audio(audio_file)
                    
                    if transcript:
                        name = extract_name_from_transcript(transcript)
                        play_text_sound(f"Nice to meet you, {name}")
                        
                        index_result = index_face(client, collection_id, image_path, name)
                        if index_result:
                            print(f"Successfully indexed face for {name}")
                            return index_result, name
                        else:
                            print("Failed to index face")
                            return None, None
                    else:
                        play_text_sound("I'm sorry, I couldn't understand that")
                        last_name = None
                    
                    os.unlink(audio_file)
                else:
                    last_name = None
                    
                return None, last_name
                
        except ClientError as e:
            print(f'Error searching faces: {e}')
            return None, last_name

def index_face(client, collection_id, image_path, name):
    try:
        with open(image_path, 'rb') as image:
            response = client.index_faces(
                CollectionId=collection_id,
                Image={'Bytes': image.read()},
                ExternalImageId=name,
                MaxFaces=1,
                QualityFilter="AUTO",
                DetectionAttributes=['ALL']
            )
            return response['FaceRecords'][0] if response['FaceRecords'] else None
    except ClientError as e:
        print(f'Error indexing face: {e}')
        return None

def main():
    set_alsa_volume()
    
    collection_id = 'collection-id'
    image_path = 'images/color_image_.png'
    last_name = None

    client = boto3.client('rekognition', aws_access_key_id=os.getenv("AWS_ACCESS_KEY_ID"),
                          aws_secret_access_key=os.getenv("AWS_SECRET_ACCESS_KEY"),
                          region_name='us-east-1')

    existing_collections = client.list_collections()['CollectionIds']
    if collection_id not in existing_collections:
        client.create_collection(CollectionId=collection_id)

    images_path = 'images'
    if not os.path.exists(images_path):
        os.makedirs(images_path)
    else:
        shutil.rmtree(images_path)
        os.makedirs(images_path)

    # Initialize camera using the new camera.py library
    try:
        camera = RealsenseCamera()
        print("Using Realsense Camera")
        get_frame = lambda: camera.get_frames()[0]  # Get the color image only
    except Exception as e_realsense:
        print(f"Failed to initialize Realsense camera: {e_realsense}")
        try:
            camera = USBCamera(index=0)
            print("Using USB Camera")
            get_frame = camera.get_frame
        except Exception as e_usb:
            print(f"Error initializing cameras: {e_usb}")
            return

    # Wait a moment for the camera to initialize
    time.sleep(2)

    try:
        while True:
            frame = get_frame()
            if frame is None:
                print("Error: Could not grab frame from camera")
                continue

            # Rotate the frame if needed
            # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            
            cv2.imwrite(image_path, frame)
            result, last_name = search_faces(client, collection_id, image_path, last_name)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
