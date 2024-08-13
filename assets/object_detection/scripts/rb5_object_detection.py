#!/usr/bin/env python3.6

import os
import time
import cv2
import json
import numpy as np
import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox
from move_robot.msg import ObjectFound
import qcsnpe as qc
from object_detection.msg import FloatList
from std_msgs.msg import String
from ros_speech_recognization.msg import data
from flask import Flask, Response
from multiprocessing import Process, Queue

CPU = 0
GPU = 1
DSP = 2

classes = {value['id'] - 1: value['name'] for value in json.load(open('/root/OBJ_WS/src/object_detection/scripts/coco_90.json', 'r')).values()}
num_classes = 90

# Global variables for detection state
detection_offset = 0.0
detection_relative_area = 0.0
detected = 0

# Global variable for the recognized label
recognized_label = None

app = Flask(__name__)
frame_queue = Queue(maxsize=1)

def preprocess_image(image, image_size):
    image = cv2.resize(image, (300, 300))
    scale = image_size / 300
    image = image.astype(np.float32)
    return image, scale

def run_inference_for_single_image(model, image):
    image_resized, scale = preprocess_image(image, 300)
    out = model.predict(image_resized)
    
    scores = np.array(out["detection_output_0_0"]).reshape((1, 20))
    boxes = np.array(out["detection_output_0_1"]).reshape((1, 20, 4))
    labels = np.array(out["detection_output_0_2"]).reshape((1, 20))
    
    return {'detection_classes': labels, 'detection_boxes': boxes, 'detection_scores': scores, 'image': image_resized}

class DLCPredictor:
    def __init__(self, model_path, dsp_delegate=DSP):
        self.model = qc.qcsnpe(model_path, [], dsp_delegate)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('rb5/image_raw', Image, queue_size=10)
        self.bbox_pub = rospy.Publisher('rb5/bounding_boxes', BoundingBoxes, queue_size=10)
        self.control_pub = rospy.Publisher('/control_robot/camera_detection', FloatList, queue_size=10)
        rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        #rospy.Subscriber('ros_speech_commands_classification/predicted_label', String, self.label_callback)
        rospy.Subscriber('whisper_ros_speech_commands_classification/predicted_label', String, self.label_callback)
 
    def label_callback(self, msg):
        global recognized_label
        recognized_label = msg.data

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = image[:, :, ::-1]
            
            output_dict = run_inference_for_single_image(self.model, image)
            
            labels = output_dict['detection_classes']
            boxes = output_dict['detection_boxes']
            scores = output_dict['detection_scores']
            original_image = output_dict['image']

            header = msg.header
            bbox_msg, Out_image = self.convert_to_bboxes(boxes, scores, labels, header, original_image)
           
            self.bbox_pub.publish(bbox_msg)
            if not frame_queue.full():
                frame_queue.put(Out_image)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")
    
    def convert_to_bboxes(self, boxes, scores, labels, header, image):
        global recognized_label
        
        bbox_msg = BoundingBoxes()
        bbox_msg.header = header
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        img_width, img_height = image.shape[1], image.shape[0]


        for i in range(boxes.shape[1]):
            if scores[0, i] > 0.3:
                box = boxes[0, i]
                x0 = int(box[1] * 300)
                y0 = int(box[0] * 300)
                x1 = int(box[3] * 300)
                y1 = int(box[2] * 300)

                bbox = BoundingBox()
                bbox.xmin = np.uint16(x0)
                bbox.ymin = np.uint16(y0)
                bbox.xmax = np.uint16(x1)
                bbox.ymax = np.uint16(y1)
                bbox.probability = float(scores[0, i])
                bbox.class_id = str(int(labels[0, i]))

                bbox.id = np.uint16(i)
                bbox.img_width = np.uint16(img_width)
                bbox.img_height = np.uint16(img_height)
                center_x = (x0 + x1) / 2.0
                center_dist = int(center_x - (img_width / 2.0))
                bbox.center_dist = center_dist

                bbox_msg.bounding_boxes.append(bbox)
                
                box = box.astype(np.int32)
                label_id = int(labels[0, i]) - 1
                label_text = f"{classes[label_id]}: {scores[0, i]*100:.1f}%"
                cv2.rectangle(image, (x0, y0), (x1, y1), (42, 42, 165), 1)
                cv2.rectangle(image, (x0, y0-20), (x0 + len(label_text) * 10, y0), (42, 42, 165), -1)
                cv2.putText(image, label_text, (x0, y0-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                detected_box_width = x1 - x0
                detected_box_height = y1 - y0
                detected_box_centerX = (x0 + x1) / 2.0
                detected_box_area = detected_box_width * detected_box_height


                if recognized_label == classes[label_id]:  # Check if recognized label matches detected label

                    detection_offset = detected_box_centerX - (image.shape[1] / 2.0)
                    detection_relative_area = detected_box_area / (image.shape[0] * image.shape[1])
                    detected = 1
                
                    # Publish FloatList message
                    msg = FloatList()
                    msg.area = detection_relative_area
                    msg.xOffset = detection_offset
                    msg.detected = detected 
                    self.control_pub.publish(msg)
                else:
                    detected = 0

        #cv2.imwrite("/root/OBJ_WS/Framedsp.jpg", image)
        image = cv2.resize(image, (495, 475))
        return bbox_msg, image

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if not frame_queue.empty():
                frame = frame_queue.get()
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
            time.sleep(0.1)

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def ros_process():
    rospy.init_node('Inference_node1')
    model_path = '/root/OBJ_WS/src/object_detection/scripts/ssd_mobilenet_quant_2.10.dlc'

    predictor = DLCPredictor(model_path)
    rospy.spin()

if __name__ == '__main__':
    ros_proc = Process(target=ros_process)
    ros_proc.start()
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader= False)
    ros_proc.join()

