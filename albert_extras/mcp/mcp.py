#!/usr/bin/env python3
# encoding: utf-8


import sys
import os

from time import sleep
import threading

from flask import Flask, Response, jsonify
from flask_cors import CORS
from flask import request

app = Flask(__name__)
CORS(app)


def gen_frames():
    
    return

@app.route('/start_trip/<trip_name>')
def start_trip(trip_name):
    

    response = {"message": "Trip " + trip_name + " started !"}
    return jsonify(response)


@app.route('/list_trips')
def list_trips():
    trip_directory = "/root/module/trips"  # Change this to your trip directory
    # List all available trips in the directory
    trips = []
    for trip_name in os.listdir(trip_directory):
        if trip_name.endswith(".trip"):
            trips.append(trip_name)

    response = {"trips": trips}
    return jsonify(response)

if __name__ == '__main__':
    app.run(threaded=True, host='0.0.0.0', port=5002)



