#!/usr/bin/env python3

import pyaudio
import wave
import sys
import time

def test_microphone():
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    
    # Print available audio devices
    print("\nAvailable Audio Input Devices:")
    for i in range(p.get_device_count()):
        dev_info = p.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0:  # Only show input devices
            print(f"Device {i}: {dev_info['name']}")
    
    # Get default input device info
    default_input = p.get_default_input_device_info()
    print(f"\nUsing default input device: {default_input['name']}")
    # Set recording parameters
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    RECORD_SECONDS = 5
    
    try:
        # Open audio stream
        stream = p.open(format=FORMAT,
                       channels=CHANNELS,
                       rate=RATE,
                       input=True,
                       frames_per_buffer=CHUNK)
        
        print("Recording 5 seconds of audio...")
        
        # Record audio
        frames = []
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK)
            frames.append(data)
            
        print("Finished recording")
        
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        
        # Save the recorded data as a WAV file
        wf = wave.open("test_microphone.wav", 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        print("Saved audio to test_microphone.wav")
        return True
        
    except Exception as e:
        print(f"Error recording audio: {e}")
        return False
    finally:
        # Terminate PyAudio
        p.terminate()

if __name__ == "__main__":
    success = test_microphone()
    sys.exit(0 if success else 1)
