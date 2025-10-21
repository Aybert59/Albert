#!/usr/bin/env python3
# encoding: utf-8


import sys
import os
import cv2 as cv
import numpy as np
from time import sleep
import threading
from ultralytics import YOLO

from flask import Flask, Response, jsonify
from flask_cors import CORS
from flask import request

app = Flask(__name__)
CORS(app)

InputURL = "http://localhost:8080/stream?topic=/camera/rgb/image_raw"  # default camera rgb
model = None
#model = YOLO("/root/module/object_models/yolo11n.engine")  # using TensorRT engine for better performance on Jetson Orin NX

# attention versions de numpy, opencv etc.. très spévcifiques... du coup on ne peut rien afficherd'ici. Juste faire un stream 


sleep (5)
cap = None
results = None
DetectionThread = None
stop_event = threading.Event()

def detect_frames():
    global cap, results

    if model is None:
        print("Model is not loaded!")
        return
    
    #try 5 times to open the video stream
    for i in range(5):
        cap = cv.VideoCapture(InputURL)  # Utilise la webcam
        if cap.isOpened():
            break
        sleep(2)    

    if not cap.isOpened():
        print("Error opening video stream or file")
        sys.exit(1)

    while True:
        if stop_event.is_set():
            print ("******************* Stopping detection thread.")
            break
        
        success, frame = cap.read()
        # pour des raisons de perf ne traiter qu'une image sur 10
        if not (cap.get(cv.CAP_PROP_POS_FRAMES) % 10 == 0):
            continue
        if not success:
            break
        results = model(frame, conf=0.25, verbose=True)
        
        sleep(0.2)  # add a small delay to avoid high CPU usage
        

def gen_frames():
    
    global results

    while True:
        if results is None:
            return

        annotated_frame = results[0].plot()

        ret, buffer = cv.imencode('.jpg', annotated_frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        sleep(0.2)  # add a small delay to avoid high CPU usage



@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/select_cam/<cam_name>')
def select_cam(cam_name):
    global InputURL, DetectionThread, stop_event

    if cam_name == "usb":
        InputURL = "http://localhost:8080/stream?topic=/usb_cam/image_raw"
    else:
        InputURL = f"http://localhost:8080/stream?topic=/camera/{cam_name}/image_raw"

    stop_event.set()
    DetectionThread.join()

    stop_event.clear()

    DetectionThread = threading.Thread(target=detect_frames)
    DetectionThread.daemon = True
    DetectionThread.start()

    response = {"message": f"Camera set to {cam_name} !"}
    return jsonify(response)

@app.route('/start_detection/<model_name>')
def start_detection(model_name):
    global DetectionThread, stop_event, model
    
    model_path = os.path.join("/root/module/object_models", model_name)
    if not os.path.exists(model_path):
        response = {"message": f"Model {model_name} does not exist!"}
        return jsonify(response)   
    

    if DetectionThread is not None and DetectionThread.is_alive():
        #response = {"message": "Detection is already running"}
        #return jsonify(response)
        stop_event.set()
        DetectionThread.join()
    
    # load the model
    print(f"Loading model {model_path}...")
    model = YOLO(model_path)

    stop_event.clear()

    DetectionThread = threading.Thread(target=detect_frames)
    DetectionThread.daemon = True
    DetectionThread.start()

    response = {"message": "Detection started !"}
    return jsonify(response)

@app.route('/stop_detection')
def stop_detection():
    global DetectionThread, stop_event

    if DetectionThread is None or not DetectionThread.is_alive():
        response = {"message": "Detection is not running!"}
        return jsonify(response)

    stop_event.set()
    DetectionThread.join()

    response = {"message": "Detection stopped !"}
    return jsonify(response)

@app.route('/list_objectmodels')
def list_object_models():
    model_directory = "/root/module/object_models"  # Change this to your model directory
    # List all available models in the directory
    models = []
    for model_name in os.listdir(model_directory):
        if model_name.endswith(".pt") or model_name.endswith(".engine"):
            models.append(model_name)

    response = {"models": models}
    return jsonify(response)

if __name__ == '__main__':
    app.run(threaded=True, host='0.0.0.0', port=5001)

#release the video stream and close all windows
cap.release()
cv.destroyAllWindows()  

#pTime = self.cTime = 0
#device = rospy.get_param("~device", "OrinNX")

