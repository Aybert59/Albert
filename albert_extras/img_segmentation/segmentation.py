#!/usr/bin/env python3
# encoding: utf-8


import sys
import cv2 as cv
import numpy as np
from time import sleep
import threading
from ultralytics import YOLO

from flask import Flask, Response, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

InputURL = "http://localhost:8080/stream?topic=/camera/rgb/image_raw"  
#model = YOLO("/ultralytics/yolo11n.pt")
model = YOLO("/ultralytics/yolo11n.engine")  # using TensorRT engine for better performance on Jetson Orin NX

# attention versions de numpy, opencv etc.. très spévcifiques... du coup on ne peut rien afficherd'ici. Juste faire un stream 


sleep (5)
cap = None
results = None
DetectionThread = None
stop_event = threading.Event()

def detect_frames():
    global cap, results

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

@app.route('/start_detection')
def start_detection():
    global DetectionThread, stop_event

    if DetectionThread is not None and DetectionThread.is_alive():
        response = {"message": "Detection is already running!"}
        return jsonify(response)
    
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

if __name__ == '__main__':
    app.run(threaded=True, host='0.0.0.0')

#release the video stream and close all windows
cap.release()
cv.destroyAllWindows()  

#pTime = self.cTime = 0
#device = rospy.get_param("~device", "OrinNX")

