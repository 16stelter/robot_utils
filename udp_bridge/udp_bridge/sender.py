#!/usr/bin/env python3

import os
import random
import socket
import struct
from queue import Empty, Full, Queue

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rosidl_runtime_py.utilities import get_message

from udp_bridge.message_handler import MessageHandler

# Fragment header (network byte order): message id, fragment index, fragment count
FRAGMENT_HEADER = struct.Struct("!IHH")

class AutoSubscriber:
    """
    A class which automatically subscribes to a topic as soon as it becomes available and buffers received messages
    in a queue.
    """

    def __init__(self, topic: str, hostname: str, queue_size: int, message_handler: MessageHandler, node: Node, targets: list[str]):
        """
        :param topic: Topic to subscribe to
        :type topic: str
        :param queue_size: How many received messages should be buffered
        :type queue_size: int
        """
        self.topic: str = topic
        self.queue: Queue = Queue(queue_size)
        self.message_handler: MessageHandler = message_handler
        self.node: Node = node
        self.timer: Timer | None = None
        self.msg_type_name: str = None
        self.hostname: str = hostname
        self.targets: list[str] = targets

        self.__subscriber: Subscription | None = None
        self.__subscribe()

    def __subscribe(self, backoff=1.0):
        """
        Try to subscribe to the set topic
        :param backoff: How long to wait until another try (capped at 30s)
        """
        if backoff > 30:
            backoff = 30

        if self.timer:
            self.timer.cancel()

        data_class = None
        for topic, msg_type_names in self.node.get_topic_names_and_types():
            if topic == self.topic:
                self.msg_type_name = msg_type_names[0]
                data_class = get_message(self.msg_type_name)
                # topic is known
                self.node.get_logger().debug(f"Want to subscribe to topic {self.topic}")
                # find out if topic is latched / transient local
                publisher_infos = self.node.get_publishers_info_by_topic(topic)
                latched = any(
                    info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for info in publisher_infos
                )
                qos_profile = QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=(
                        DurabilityPolicy.TRANSIENT_LOCAL
                        if latched
                        else DurabilityPolicy.VOLATILE
                    ),
                )

                self.__subscriber = self.node.create_subscription(
                    data_class,
                    self.topic,
                    lambda msg: self.__message_callback(msg, latched=latched),
                    qos_profile,
                )
                self.node.get_logger().debug(f"Subscribed to topic {self.topic}")
                return

        # topic is not yet known
        if backoff > 10:
            logging_severity = LoggingSeverity.WARN
        else:
            logging_severity = LoggingSeverity.DEBUG
        self.node.get_logger().log(
            f"Topic {self.topic} is not yet known. Retrying in {backoff} seconds", logging_severity
        )
        self.timer = self.node.create_timer(backoff, lambda: self.__subscribe(backoff * 1.2))

    def __message_callback(self, data, latched=False):
        encrypted_msg = self.message_handler.encrypt_and_encode(
            {
                "data": serialize_message(data),
                "topic": self.topic,
                "msg_type_name": self.msg_type_name,
                "hostname": self.hostname,
                "latched": latched,
            }
        )

        try:
            self.queue.put_nowait(encrypted_msg)
        except Full:
            try:
                self.queue.get_nowait()
                self.queue.put_nowait(encrypted_msg)
            except (Empty, Full):
                pass

        # for latched messages, republish them every ten seconds because we cannot latch on the other side
        if latched:
            if self.timer:
                self.timer.cancel()
            self.timer = self.node.create_timer(10.0, lambda: self.__message_callback(data, latched=True))

class UdpBridgeSender:
    def __init__(self, node: Node):
        node.declare_parameter("config_file", os.path.join(get_package_share_directory("udp_bridge"), "config", "udp_bridge.yaml"))
        config_file = node.get_parameter("config_file").value
        try:
            with open(config_file, "r") as f:
                self.params = yaml.safe_load(f)
                node.get_logger().info(f"Successfully loaded config from: {config_file}")
                node.get_logger().info(f"Config contents: {self.params}")
        except Exception as e:
            node.get_logger().error(f"Failed to load config from: {config_file} with error: {e}")
            raise e

        self.node = node
        self.freq = self.params["send_frequency"]
        self.topics = self.params["topics"]
        self.port = self.params["port"]

        self.max_packet_size: int = self.params.get("max_packet_size", 1400) # 1400 bytes is smaller than the typical MTU of 1500 bytes, leaving some safety margin
        self.next_msg_id: int = random.randint(0, 0xFFFFFFFF)

        self.sock = self.setup_udp_socket()
        hostname = self.params["hostname"]
        max_queue_size = self.params["sender_queue_max_size"]
        message_handler = self.setup_message_handler()
        self.subscribers: list[AutoSubscriber] = []
        for topic_config in self.topics:
            topic = topic_config["name"]
            targets = topic_config["target_ips"]
            self.subscribers.append(AutoSubscriber(topic, hostname, max_queue_size, message_handler, node, targets))

    def setup_udp_socket(self) -> socket.socket:
        sock = socket.socket(type=socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.params.get("send_buffer_size", 8 * 1024 * 1024))
        return sock

    def setup_message_handler(self) -> MessageHandler:
        encryption_key: str | None = None
        if self.params.get("encryption_key"):
            encryption_key = self.params["encryption_key"]

        return MessageHandler(encryption_key)

    def split_into_packets(self, data: bytes) -> list[bytes]:
        """
        Split a message into packets of at most max_packet_size bytes (fragment header included).
        A message which fits into one packet results in a single packet.
        """
        chunk_size = self.max_packet_size - FRAGMENT_HEADER.size
        count = max(1, -(-len(data) // chunk_size))
        if count > 0xFFFF:
            raise ValueError(
                f"message of {len(data)} bytes needs {count} fragments, but at most {0xFFFF} are supported"
            )

        msg_id = self.next_msg_id
        self.next_msg_id = (self.next_msg_id + 1) & 0xFFFFFFFF

        return [
            FRAGMENT_HEADER.pack(msg_id, index, count) + data[index * chunk_size : (index + 1) * chunk_size]
            for index in range(count)
        ]

    def send_messages_in_queue(self):
        for subscriber in self.subscribers:
            try:
                data = subscriber.queue.get_nowait()
            except Empty:
                continue

            try:
                packets = self.split_into_packets(data)
            except ValueError as e:
                self.node.get_logger().error(f"Could not split packets for topic {subscriber.topic}: {e}")
                continue

            for target in subscriber.targets:
                try:
                    for packet in packets:
                        self.sock.sendto(packet, (target, self.port))
                except Exception as e:
                    self.node.get_logger().error(
                        f"Could not send data of topic {subscriber.topic} to {target} with error {str(e)}"
                    )


def main():
    rclpy.init()

    node = Node("udp_bridge_sender")

    sender = UdpBridgeSender(node)

    exec = SingleThreadedExecutor()
    exec.add_node(node)
    node.create_timer((1 / sender.freq), sender.send_messages_in_queue)
    exec.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()