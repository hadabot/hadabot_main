# Copyright AF386 Group LLC 2019

import json
import logging


logger = logging.getLogger(__name__)


###############################################################################
class Message:
    ###########################################################################
    def __init__(self, values=None):
        self._values = values

    ###########################################################################
    @property
    def str_message(self):
        """
        If values of message is a dict, then convert it into a JSON string
        """
        if isinstance(self._values, dict):
            return json.dumps(self._values)
        else:
            return self._values

    @property
    def values(self):
        return self._values

###############################################################################


class Topic:
    """Publish and/or subscribe to a topic in ROS.

    Args:
        ros (:class:`.Ros`): Instance of the ROS connection.
        name (:obj:`str`): Topic name, e.g. ``/cmd_vel``.
        message_type (:obj:`str`): Message type, e.g. ``std_msgs/String``.
        compression (:obj:`str`): Type of compression to use, e.g. `png`.
            Defaults to `None`.
        throttle_rate (:obj:`int`): Rate (in ms between messages) at which to
            throttle the topics.
        queue_size (:obj:`int`): Queue size created at bridge side for
            re-publishing webtopics.
        latch (:obj:`bool`): True to latch the topic when publishing,
            False otherwise.
        queue_length (:obj:`int`): Queue length at bridge side used when
            subscribing.
    """
    SUPPORTED_COMPRESSION_TYPES = ('png', 'none')

    ###########################################################################
    def __init__(self, ros, name, message_type,
                 compression=None, latch=False, throttle_rate=0,
                 queue_size=100, queue_length=0):
        self._ros = ros
        self._name = name
        self._message_type = message_type
        self._compression = compression
        self._latch = latch
        self._throttle_rate = throttle_rate
        self._queue_size = queue_size
        self._queue_length = queue_length

        self._subscribe_id = None
        self._advertise_id = None

    ###########################################################################
    @property
    def is_advertised(self):
        """Indicate if the topic is currently advertised or not.

        Returns:
            bool: True if advertised as publisher of this topic,
                  False otherwise.
        """
        return self._advertise_id is not None

    ###########################################################################
    @property
    def is_subscribed(self):
        """Indicate if the topic is currently subscribed or not.

        Returns:
            bool: True if subscribed to this topic, False otherwise.
        """
        return self._subscribe_id is not None

    ###########################################################################
    def subscribe(self, callback):
        """Register a subscription to the topic.

        Every time a message is published for the given topic,
        the callback will be called with the message object.

        Args:
            callback: Function to be called when messages of this topic are
                      published.
        """
        # Avoid duplicate subscription
        if self._subscribe_id:
            return

        self._subscribe_id = 'subscribe:%s:%d' % (
            self._name, self._ros.id_counter)

        sub_msg = Message({
            'op': 'subscribe',
            'id': self._subscribe_id,
            'type': self._message_type,
            'topic': self._name,
            'compression': self._compression,
            'throttle_rate': self._throttle_rate,
            'queue_length': self._queue_length
        })

        self._ros.on(self._name, callback)
        self._ros.send_on_ready(sub_msg)

    ###########################################################################
    def publish(self, message):
        """Publish a message to the topic.

        Args:
            message (:class:`.Message`): ROS Bridge Message to publish.
        """
        if self.is_advertised is False:
            self.advertise()

        publish_id = 'publish:%s:%d' % (self._name, self._ros.id_counter)

        pub_msg = Message({
            "op": "publish",
            "id":  publish_id,
            "topic": self._name,
            "msg": message.values,
            "latch": self._latch
        })

        try:
            rval = self._ros.send_on_ready(pub_msg)
            if rval is not None:
                logger.error("Error publishing {}".format(pub_msg.str_message))
        except Exception as ex:
            # Need to better handle this
            logger.error("Send request failed {}".format(str(ex)))
            raise ex
            pass

    ###########################################################################
    def advertise(self):
        if self.is_advertised:
            logger.error("We already advertised {}".format(self._name))

        self._advertise_id = 'advertise:%s:%d' % (
            self._name, self._ros.id_counter)

        adv_msg = Message({
            "op": "advertise",
            "id": self._advertise_id,
            "type": self._message_type,
            "topic": self._name,
            "latch": self._latch,
            "queue_size": self._queue_size
        })

        logger.info("Advertising: {}".format(adv_msg.str_message))
        rval = self._ros.send_on_ready(adv_msg)
        logger.info("Done advertising")

        if rval is not None:
            logger.info("Error advertising: {}".format(adv_msg.str_message))
            self._advertise_id = None
