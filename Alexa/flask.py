#author Nikhil Kanamarla
from flask import Flask
from flask_ask import Ask, request, statement, question, session, context

app = Flask(__name__)
ask = Ask(app, "/ud smart city")

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=4000, ssl_context=('/home/themainframe/catkin_ws/src/flask_ask_ros/config/ssl_keys', '/home/themainframe/catkin_ws/src/flask_ask_ros/config/ssl_keys'))
