#!/usr/bin/env python3
# encoding: utf-8



import sys
import cv2 as cv
import numpy as np
from time import sleep

from ultralytics import YOLO

from flask import Flask, Response

app = Flask(__name__)

InputURL = "http://localhost:8080/stream?topic=/camera/rgb/image_raw"  
model = YOLO("/ultralytics/yolo11n.pt")

# attention versions de numpy, opencv etc.. très spévcifiques... du coup on ne peut rien afficherd'ici. Juste faire un stream 

sleep (5)
cap = None
#open the video stream and check if it's opened


def gen_frames():
    cap = cv.VideoCapture(InputURL)  # Utilise la webcam
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:

            # pour des raisons de perf ne traiter qu'une image sur 10
            if (cap.get(cv.CAP_PROP_POS_FRAMES) % 10 == 0):
                results = model(frame)
                annotated_frame = results[0].plot()

                ret, buffer = cv.imencode('.jpg', annotated_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')



@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0')

#release the video stream and close all windows
cap.release()
cv.destroyAllWindows()  

#pTime = self.cTime = 0
#device = rospy.get_param("~device", "OrinNX")

