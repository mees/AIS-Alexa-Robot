import logging
import os
import queue
import threading
import time
from typing import Optional
import uuid

from flask import Flask
from flask_ask import Ask, question, session, statement
from flask_ask.models import _Response
import roslibpy
from twisted.internet import reactor

app = Flask(__name__)
ask = Ask(app, "/")
logger = logging.getLogger("flask_ask")
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())

ROS_HOST = os.getenv("ROS_HOST", "172.17.0.1")
ROS_WEBSOCKET_PORT = int(os.getenv("ROS_WEBSOCKET_PORT", 9090))
ROS_COMMAND_TOPIC = os.getenv("ROS_COMMAND_TOPIC", "/alexa/command")
ROS_REPLY_TOPIC = os.getenv("ROS_REPLY_TOPIC", "/alexa/reply")
ROS_DEBUG_TOPIC = os.getenv("ROS_DEBUG_TOPIC", "/alexa/debug")

ROS_STATE = dict()

DEFAULT_REPROMPT = "What else do you want?"
MAX_REPLY_TIMEOUT = 7
# for simplicity, we use strings in both queues to not introduce new ROS message types
# each string contains the command ID and the content, separated by the separator
# the reply also contains a third field whether disambiguation is required (0 or 1)
MESSAGE_SEPARATOR = "@@"
USER_COMMAND_QUEUE = queue.Queue()
ROS_REPLY_QUEUE = queue.Queue()


class RosReply(object):
    def __init__(self, command_uuid: str, response_text: str, require_disambiguation: bool):
        self.command_uuid = command_uuid
        self.response_text = response_text
        self.require_disambiguation = require_disambiguation

    @classmethod
    def from_reply_str(cls, reply_str: str):
        command_uuid, response_text, require_disambiguation = reply_str.split(MESSAGE_SEPARATOR)
        # expect 0 or 1
        require_disambiguation = bool(int(require_disambiguation))
        return RosReply(command_uuid, response_text, require_disambiguation)


class CanFulfillResponse(_Response):
    def __init__(self, can_fulfill, slot_infos):
        super().__init__("")
        can_fulfill_msg = "YES" if can_fulfill else "NO"

        self._response = {"canFulfillIntent": {"canFulfill": can_fulfill_msg, "slots": []}}
        for cur_slot in slot_infos:
            if "can_fulfill" in slot_infos[cur_slot] and slot_infos[cur_slot]["can_fulfill"]:
                slot_can_fulfill = "YES"
            else:
                slot_can_fulfill = "NO"
            if "can_understand" in slot_infos[cur_slot] and slot_infos[cur_slot]["can_understand"]:
                slot_can_understand = "YES"
            else:
                slot_can_understand = "NO"
            self._response["canFulfillIntent"]["slots"][cur_slot] = {
                "canFulfillIntent": slot_can_fulfill,
                "canUnderstand": slot_can_understand,
            }


@ask.launch
def launch():
    speech_output = "Welcome to AIS alexa."
    reprompt_text = 'Please start a command with "robot", such as "robot, grab me a beer"'
    return question(speech_output).reprompt(reprompt_text)


@ask.intent("RobotCommandIntent")
def receive_robot_command(command):
    logger.info(f"received command {command}")
    if "command_uuid" not in session.attributes or not session.attributes["command_uuid"]:
        session.attributes["command_uuid"] = str(uuid.uuid4())
    command_uuid = session.attributes["command_uuid"]
    command_msg = MESSAGE_SEPARATOR.join([command_uuid, command])
    USER_COMMAND_QUEUE.put_nowait(command_msg)

    reply_msg = None
    try:
        reply_uuid = None
        start_time = time.time()
        while reply_uuid != command_uuid:
            if reply_uuid is not None:
                logger.warning(f"incorrect reply UUID: expected {command_uuid}, got {reply_uuid}")
                # FIXME we just discard non-matches, so this may fail for out-of-order replies
            remaining_time = MAX_REPLY_TIMEOUT - (time.time() - start_time)
            reply_msg: Optional[RosReply] = ROS_REPLY_QUEUE.get(block=True, timeout=remaining_time)
            logger.info(f"received reply {reply_msg}")
            if reply_msg is not None:
                reply_uuid = reply_msg.command_uuid
    except queue.Empty:
        logger.warning(f"did not receive reply to {command_msg} within timeout")
    # we might also send a progressive response here...
    # https://developer.amazon.com/de/docs/custom-skills/send-the-user-a-progressive-response.html
    if reply_msg is None:
        reply_msg = RosReply(
            command_uuid, "The robot seems to be tired. What else do you want?", require_disambiguation=False
        )

    if reply_msg.require_disambiguation:
        response_text = f'{reply_msg.response_text}. Please start your answer with "Robot".'
        response = (
            question(response_text)
            .simple_card(
                "Disambiguation", content=f"Received command {command}, disambiguating with {reply_msg.response_text}"
            )
            .reprompt(reply_msg.response_text)
        )
    else:
        session.attributes["command_uuid"] = ""  # new UUID for next command
        response_text = f"{reply_msg.response_text}. {DEFAULT_REPROMPT}"
        response = (
            question(response_text)
            .simple_card(
                "Successful command",
                content=f"Received command {command}, {reply_msg.response_text}, waiting for next one",
            )
            .reprompt(DEFAULT_REPROMPT)
        )

    return response


@ask.can_fullfill_intent
def check_valid_intents(intent):
    logger.info(f"received CanFullfillIntentRequest: {intent}")
    logger.info(f"intent name {intent.name}")
    slot_infos = dict()
    for cur_slot_name, cur_slot_dict in intent.slots.items():
        logger.info(f"got slot name {cur_slot_name} with value {cur_slot_dict}")
        slot_infos[cur_slot_name] = {"can_fulfill": True, "can_understand": True}

    return CanFulfillResponse(can_fulfill=True, slot_infos=slot_infos)


def start_publish_loop():

    logger.info("ROS client connected, starting publish loop")
    i = 0
    while True:
        if "client" in ROS_STATE and ROS_STATE["client"].is_connected:
            if "publisher" not in ROS_STATE:
                ROS_STATE["publisher"] = roslibpy.Topic(ROS_STATE["client"], ROS_COMMAND_TOPIC, "std_msgs/String")
                ROS_STATE["publisher"].advertise()
                logger.info(f"advertised topic {ROS_COMMAND_TOPIC}")
                ROS_STATE["debug_publisher"] = roslibpy.Topic(ROS_STATE["client"], ROS_DEBUG_TOPIC, "std_msgs/String")
                ROS_STATE["debug_publisher"].advertise()
                logger.info("advertised topic /alexa/debug")
                i = 0

            if "subscriber" not in ROS_STATE:
                ROS_STATE["subscriber"] = roslibpy.Topic(ROS_STATE["client"], ROS_REPLY_TOPIC, "std_msgs/String")
                ROS_STATE["subscriber"].subscribe(process_ros_reply)

            while not USER_COMMAND_QUEUE.empty():
                try:
                    command = USER_COMMAND_QUEUE.get_nowait()
                    ROS_STATE["publisher"].publish(roslibpy.Message({"data": command}))
                except queue.Empty:
                    logger.warning("concurrent reads to command queue")
            if i % 50 == 0:
                ROS_STATE["debug_publisher"].publish(roslibpy.Message({"data": f"alive ({i // 50})"}))
            i += 1
            if i > 1e10:
                i = 0
        else:
            logger.info("ROS client disconnected")
            stop_ros_connection()
            break

        time.sleep(0.1)
    # if reactor.running:
    #     reactor.stop()


def process_ros_reply(ros_reply_msg):
    try:
        ROS_REPLY_QUEUE.put_nowait(RosReply.from_reply_str(ros_reply_msg["data"]))
    except ValueError as e:
        logger.error(f"error parsing {ros_reply_msg}: {e}")
        ROS_REPLY_QUEUE.put_nowait(None)


def start_ros_connection():
    while True:
        stop_ros_connection()
        logger.info(f"trying to establish ROS connection on websocket {ROS_HOST}:{ROS_WEBSOCKET_PORT}")
        ROS_STATE["client"] = roslibpy.Ros(ROS_HOST, ROS_WEBSOCKET_PORT)
        ROS_STATE["client"].on_ready(start_publish_loop, run_in_thread=True)
        # ROS_STATE['client'].run_forever()
        # def check_connection():
        #     if reactor.running and not ROS_STATE['client'].is_connected:
        #         logger.info('no connection found')
        #         # ROS_STATE['client'].terminate()
        #
        # loop = task.LoopingCall(check_connection)
        # loop.start(5.)
        reactor.run(installSignalHandlers=False)
        logger.info("rosclient main loop terminated")
        time.sleep(1)


def stop_ros_connection():
    if "publisher" in ROS_STATE and ROS_STATE["publisher"].is_advertised:
        ROS_STATE["publisher"].unadvertise()
        ROS_STATE["debug_publisher"].unadvertise()
        del ROS_STATE["publisher"]
        del ROS_STATE["debug_publisher"]
    if "subscriber" in ROS_STATE:
        ROS_STATE["subscriber"].unsubscribe()
        del ROS_STATE["subscriber"]
    if "client" in ROS_STATE:
        ROS_STATE["client"].terminate()
    ROS_STATE.clear()


@ask.intent("AMAZON.StopIntent")
def stop():
    return statement("Goodbye")


@ask.intent("AMAZON.CancelIntent")
def cancel():
    return statement("Goodbye")


@ask.session_ended
def session_ended():
    return "{}", 200


if __name__ == "__main__":
    if "ASK_VERIFY_REQUESTS" in os.environ:
        verify = str(os.environ.get("ASK_VERIFY_REQUESTS", "")).lower()
        if verify == "false":
            app.config["ASK_VERIFY_REQUESTS"] = False
    ros_thread = threading.Thread(target=start_ros_connection)
    ros_thread.daemon = True
    ros_thread.start()
    app.run(host="0.0.0.0", port=80, debug=False)
