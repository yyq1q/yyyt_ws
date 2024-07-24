#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from flask import Flask, request, jsonify
from threading import Thread

app = Flask(__name__)

# ROS Publisherの設定
pub = rospy.Publisher('mode_msgs', String, queue_size=10)
rospy.init_node('message_receiver', anonymous=True)

@app.route('/message', methods=['POST'])
def receive_message():
    if request.is_json:
        data = request.get_json()
        message = data.get('message')
        if message:
            rospy.loginfo(f"Received message: {message}")
            # ROS topicとしてメッセージをパブリッシュ
            pub.publish(String(message))
            return jsonify({"status": "success", "message": "Message received and published"}), 200
        else:
            return jsonify({"status": "error", "message": "No message found in request"}), 400
    else:
        return jsonify({"status": "error", "message": "Request must be JSON"}), 400

def run_flask():
    app.run(host='0.0.0.0', port=5555)

if __name__ == '__main__':
    try:
        # FlaskサーバーをRosのメインループとは別のスレッドで実行
        Thread(target=run_flask).start()
        rospy.loginfo("Flask server started on port 5555")
        
        # ROSのメインループを開始
        rospy.spin()
    except rospy.ROSInterruptException:
        pass