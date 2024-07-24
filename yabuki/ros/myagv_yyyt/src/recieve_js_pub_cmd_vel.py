#!/usr/bin/env python3

from flask import Flask, request, jsonify
import rospy
from geometry_msgs.msg import Twist

app = Flask(__name__)

# ROSノードを初期化します
rospy.init_node('flask_ros_publisher', anonymous=True)
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

@app.route('/data', methods=['POST'])
def receive_data():
    data = request.json
    rospy.loginfo(f"Received data: {data}")
    
    # データをTwistメッセージとしてpublishします
    msg = Twist()
    msg.linear.x =  data.get('Y_L', 0.0)
    msg.linear.y = -data.get('X_L', 0.0)
    msg.linear.z = 0.0  # Z方向の速度はデフォルトで0とします
    msg.angular.x = 0.0  # 回転速度はデフォルトで0とします
    msg.angular.y = 0.0
    msg.angular.z = -data.get('X_R', 0.0)
    
    publisher.publish(msg)
    
    return jsonify({"status": "success"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
