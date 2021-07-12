# Copyright AF386 Group LLC 2019

from .uwebsockets import client
import json
import logging


logger = logging.getLogger(__name__)


CONNECTION_TIMEOUT = 10


###############################################################################
class Ros:

    ###########################################################################
    def __init__(self, host, port=None, is_secure=False):
        self._id_counter = 0
        self._host = host
        self._port = 9090 if port is None else port
        self._ws = None

        self._event_callbacks = {}

        self.connect()

    ###########################################################################
    @property
    def id_counter(self):
        """Generate an auto-incremental ID starting from 1.

        Returns:
            int: An auto-incremented ID.
        """
        self._id_counter += 1
        return self._id_counter

    ###########################################################################
    @property
    def is_connected(self):
        """Indicate if the ROS connection is open or not.

        Returns:
            bool: True if connected to ROS, False otherwise.
        """
        return self._ws is not None

    ###########################################################################
    def connect(self):
        if self.is_connected:
            return

        logger.info(
            "Connecting websocket: {}:{}".format(self._host, self._port))
        self._ws = client.connect(
            'ws://{}:{}'.format(self._host, self._port))

        self._ws.setblocking(1)
        self._ws_blocking = True

        logger.info("Connected")

    ###########################################################################
    def on(self, event_name, callback):
        """Add a callback to an arbitrary named event.

        Args:
            event_name (:obj:`str`): Name of the event to which to subscribe.
            callback: Callable function to be executed when the event is triggered.
        """
        if event_name not in self._event_callbacks:
            self._event_callbacks[event_name] = []
        self._event_callbacks[event_name].append(callback)

    ###########################################################################
    def send_on_ready(self, message):
        """Send message to the ROS Master once the connection is established.

        If a connection to ROS is already available, the message is sent immediately.

        Args:
            message (:class:`.Message`): ROS Bridge Message to send.
        """
        self._ws.send(message.str_message)

    ###########################################################################
    def run_once(self, timeout=CONNECTION_TIMEOUT):
        # Make sure our socket is not set to blocking else run_once
        # may never return if nothing received
        if self._ws_blocking:
            self._ws.setblocking(0)
            self._ws_blocking = False

        # Get any responses from server
        rval = self._ws.recv()

        if rval == "":
            return

        logger.info("Received {}".format(rval))
        frame = json.loads(rval)

        if frame["op"] == "publish":
            topic = frame["topic"]
            msg = frame["msg"]

            if topic not in self._event_callbacks:
                logger.error(
                    "Why did we receive this publish frame even though "
                    "we never subscribed to it - {}".format(rval))

            if len(self._event_callbacks[topic]) == 0:
                logger.error(
                    "There are no callbacks registered for this publish "
                    "frame - {}".format(rval))

            # Make callbacks
            for cb in self._event_callbacks[topic]:
                cb(msg)

    ###########################################################################
    def run_forever(self):
        pass

    ###########################################################################
    def terminate(self):
        self._ws.close()
        self._ws = None
