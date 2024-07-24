from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route('/')
def index():
    return "WebSocket Server Running"  # または適切なHTMLを返す

# 既存のSocketIOイベントハンドラー
@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('joystick_data')
def handle_joystick_data(data):
    print('Received joystick data:', data)
    # データ処理ロジック

if __name__ == '__main__':
    print("WebSocket server starting...")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)