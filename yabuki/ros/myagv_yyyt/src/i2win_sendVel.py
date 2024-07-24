#!/usr/bin/env python3

from flask import Flask, request, jsonify
import zmq
import threading
import time
import numpy as np

app = Flask(__name__)

data_common = []
data_lock = threading.Lock()

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://192.168.5.3:7777")

@app.route('/data', methods=['POST'])
def receive_data():
    global data_common
    data = request.json
    print(f'Received data: {data}')
    cmd_vel_x   =  data.get('Y_L', 0.0)
    cmd_vel_y   = -data.get('X_L', 0.0)
    cmd_vel_yaw = -data.get('X_R', 0.0)
    #data_common = [data.get('Y_L', 0.0), -data.get('X_L', 0.0), -data.get('X_R', 0.0)]
    with data_lock:
        data_common = [cmd_vel_x, cmd_vel_y, cmd_vel_yaw]

    #     data_packet = np.array(data_common, dtype=np.float32)
    # serialized_data = data_packet.tobytes()
    # socket.send(serialized_data)
    return jsonify({"status": "success"}), 200

def send_data():
    while True:
        with data_lock:
            data_packet = np.array(data_common, dtype=np.float32)
        serialized_data = data_packet.tobytes()
        socket.send(serialized_data)
        time.sleep(0.01)

thread1 = threading.Thread(target=send_data)
thread1.start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=1515)

thread1.join()