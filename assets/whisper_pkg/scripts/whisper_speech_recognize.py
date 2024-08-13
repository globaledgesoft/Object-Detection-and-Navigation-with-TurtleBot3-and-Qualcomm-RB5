#!/usr/bin/env python3.8

import os
import rospy
import whisper
import numpy as np
from timeit import default_timer as timer
import tflite_runtime.interpreter as tflite
import sounddevice as sd
import scipy.io.wavfile as wav
from move_robot.msg import ObjectFound
from std_msgs.msg import String, Float32
import json
import io
import time

# Variable to keep track of the last command label
last_command_label = None

# Define the path to the TFLite model
tflite_model_path = '/root/WHISPER_WS/src/whisper_pkg/scripts/whisper-base.tflite'

# -- Settings
ROOT = os.path.dirname(os.path.abspath(__file__)) + "/"
ROS_TOPIC_PREDICTED_LABEL = "whisper_ros_speech_commands_classification/predicted_label"
ROS_TOPIC_PREDICTED_PROBABILITY = "whisper_ros_speech_commands_classification/predicted_probability"
COCO_JSON_PATH = os.path.join(ROOT, 'coco_90.json')
# Load COCO JSON
with open(COCO_JSON_PATH, 'r') as f:
    coco_data = json.load(f)
coco_labels = {cat['name'].lower() for cat in coco_data['categories']}


# Create an interpreter to run the TFLite model
interpreter = tflite.Interpreter(model_path=tflite_model_path)

# Allocate memory for the interpreter
interpreter.allocate_tensors()

# Get the input and output tensors
input_tensor = interpreter.get_input_details()[0]['index']
output_tensor = interpreter.get_output_details()[0]['index']

class Publishers(object):
    ''' ROS publisher for predicted label and probability '''

    def __init__(self):
        self._pub_label = rospy.Publisher( ROS_TOPIC_PREDICTED_LABEL, String, queue_size=10)
        self._pub_prob = rospy.Publisher( ROS_TOPIC_PREDICTED_PROBABILITY, Float32, queue_size=10)
 
    def publish(self, label, prob):
        self._pub_label.publish(label)
        self._pub_prob.publish(prob)# Publish the custom message

class TimerPrinter(object):
    ''' A class for printing message with time control:
        If previous `print` is within T_gap seconds, 
        then the currrent `print` prints nothing. 
    '''

    def __init__(self, print_period):
        self._prev_time = -999.0
        self._t_print_gap = print_period

    def print(self, s):
        if time.time() - self._prev_time >= self._t_print_gap:
            self._prev_time = time.time()
            print(s)

def record_audio(duration, fs):
    print("Recording...")
    audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float32', blocking=True, device=0)
    sd.wait()
    print("Recording complete.")
    return audio

def save_wav(file_path, audio, fs):
    wav.write(file_path, fs, (audio * 32767).astype(np.int16))

def transcribe_audio():
    duration = 6  # seconds
    fs = 16000  # sample rate
    audio = record_audio(duration, fs)
    
    # Save the recorded audio to a temporary file
    temp_audio_path = '/root/WHISPER_WS/src/whisper_pkg/scripts/temp_audio.wav'
    save_wav(temp_audio_path, audio, fs)

    inference_start = timer()

    print(f'Calculating mel spectrogram...')
    mel_from_file = whisper.audio.log_mel_spectrogram(temp_audio_path)

    input_data = whisper.audio.pad_or_trim(mel_from_file, whisper.audio.N_FRAMES)
    input_data = np.expand_dims(input_data, 0)

    print("Invoking interpreter ...")
    interpreter.set_tensor(input_tensor, input_data)
    interpreter.invoke()

    output_data = interpreter.get_tensor(output_tensor)

    wtokenizer = whisper.tokenizer.get_tokenizer(True, language="ja")

    print("Converting tokens ...")
    transcribed_text = ""
    for token in output_data:
        token[token == -100] = wtokenizer.eot
        text = wtokenizer.decode(token)
        transcribed_text += text

    print("\nInference took {:.2f}s ".format(timer() - inference_start))
    print(f"Transcription: {transcribed_text}")

    return transcribed_text

def validate_transcript(transcript):
    '''Validate if the transcript contains any command phrases.'''
    print("Transcript", transcript)
    transcript_lower = transcript.lower()
    for label in coco_labels:
        if label in transcript_lower:
            return label
    return None

def object_found_callback(data):
    global last_command_label
    if data.detected:
        last_command_label = None  # Reset the last command label
        print("Object found! Ready to listen for new commands.")

def inference_from_microphone():
    # ROS publishers
    publishers = Publishers()
    # Timer to manage audio recording intervals
    timer_printer = TimerPrinter(print_period=2.0)  # for print
    
    global last_command_label
    last_command_label = None  # Initialize command label

    # ROS subscriber to listen for object found messages
    rospy.Subscriber('/Object_found', ObjectFound, object_found_callback)  # Subscribe to the custom message topic

    while not rospy.is_shutdown():
        timer_printer.print("Usage: Keep pressing down 'R' to record audio")

        if last_command_label is None:
            transcript = transcribe_audio()
            command_label = validate_transcript(transcript)
            if command_label:
                publishers.publish(command_label, 1.0)  # Dummy probability
                last_command_label = command_label
            else:
                print(f"No valid command found in transcript: {transcript}")
        else:
            publishers.publish(last_command_label, 1.0)  # Dummy probability

        rospy.sleep(0.1)  # Sleep for a bit before the next iteration

if __name__ == "__main__":
    rospy.init_node("whisper_transcription_node")
    inference_from_microphone()
    rospy.spin()
