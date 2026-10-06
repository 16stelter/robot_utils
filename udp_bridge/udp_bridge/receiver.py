#!/usr/bin/env python3

import os
import socket
import struct
import time
from collections import OrderedDict
from threading import Thread

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from udp_bridge.message_handler import MessageHandler

# Fragment header (network byte order): message id, fragment index, fragment count
FRAGMENT_HEADER = struct.Struct("!IHH")


class UdpBridgeReceiver:
    def __init__(self, node: Node):
        self.node = node
        node.declare_parameter("config_file", os.path.join(get_package_share_directory("udp_bridge"), "config", "udp_bridge.yaml"))
        config_file = node.get_parameter("config_file").value
        with open(config_file, "r") as f:
            self.params = yaml.safe_load(f)
        port: int = self.params["port"]
        self.node.get_logger().info(f"Initializing udp_bridge on port {port}")

        self.sock = socket.socket(type=socket.SOCK_DGRAM)
        requested_rcvbuf = self.params.get("receive_buffer_size", 8 * 1024 * 1024)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, requested_rcvbuf)
        actual_rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        # Linux reports twice the effective value, so this only triggers if it is clearly below the request
        if actual_rcvbuf < requested_rcvbuf:
            self.node.get_logger().warning(
                f"Receive buffer is {actual_rcvbuf} bytes instead of the requested {requested_rcvbuf}. "
                f"Consider raising net.core.rmem_max if large messages get lost."
            )
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(1)

        self.known_senders: list[str] = []
        self.publishers = {}

        encryption_key: str | None = None
        if self.params.get("encryption_key"):
            encryption_key = self.params["encryption_key"]

        self.message_handler = MessageHandler(encryption_key)

        self.pending: OrderedDict[tuple, dict] = OrderedDict()
        self.reassembly_timeout: float = self.params.get("reassembly_timeout", 2.0)
        self.reassembly_max_pending: int = self.params.get("reassembly_max_pending", 64)
        self.max_message_size: int = self.params.get("max_message_size", 64 * 1024 * 1024)
        self.last_purge: float = time.monotonic()

    def recv_message(self):
        """
        Receive packets from the network, reassemble them into messages, process those and publish them into ROS
        """
        while rclpy.ok():
            try:
                # 65535 is the upper limit for the size because of network properties
                packet, source = self.sock.recvfrom(65535)
                try:
                    msg = self.reassemble(packet, source)
                except ValueError as e:
                    self.node.get_logger().warning(f"Ignoring invalid packet from {source}: {e}")
                    return
                if msg is not None:
                    self.handle_message(msg)
            except socket.timeout:
                pass

    def reassemble(self, packet: bytes, source) -> bytes | None:
        """
        Add a fragment to its message.

        :param packet: Raw packet including the fragment header
        :param source: Address of the sender, as returned by recvfrom
        :return: The complete message once its last fragment arrived, otherwise None
        :raises ValueError: If the packet is not a valid fragment
        """
        if len(packet) < FRAGMENT_HEADER.size:
            raise ValueError("packet is shorter than the fragment header")

        msg_id, index, count = FRAGMENT_HEADER.unpack_from(packet)
        if count == 0 or index >= count:
            raise ValueError(f"invalid fragment header (index {index}, count {count})")

        chunk = packet[FRAGMENT_HEADER.size :]

        if count == 1:
            return chunk

        key = (source, msg_id)
        partial = self.pending.get(key)

        if partial is not None and partial["count"] != count:
            # same id but a different layout, so the old one must be stale
            del self.pending[key]
            partial = None

        if partial is None:
            if len(self.pending) >= self.reassembly_max_pending:
                self.pending.popitem(last=False)  # drop the oldest incomplete message
            partial = {"count": count, "chunks": [None] * count, "received": 0, "size": 0, "first_seen": time.monotonic()}
            self.pending[key] = partial

        if partial["chunks"][index] is not None:
            return None  # duplicate fragment

        partial["chunks"][index] = chunk
        partial["received"] += 1
        partial["size"] += len(chunk)

        if partial["size"] > self.max_message_size:
            del self.pending[key]
            return None

        if partial["received"] < partial["count"]:
            return None

        del self.pending[key]
        return b"".join(partial["chunks"])

    def purge_pending(self):
        """
        Drop incomplete messages which are older than the timeout (e.g. because a fragment was lost).
        Cheap to call often, it only does work every half second.
        """
        now = time.monotonic()
        if now - self.last_purge < 0.5:
            return
        self.last_purge = now

        expired = 0
        # entries are ordered by first_seen, so we can stop at the first one which is still fresh
        while self.pending:
            key, partial = next(iter(self.pending.items()))
            if now - partial["first_seen"] <= self.reassembly_timeout:
                break
            del self.pending[key]
            expired += 1

        if expired:
            self.node.get_logger().warning(
                f"Dropped {expired} incomplete message(s) because fragments were lost or arrived too late"
            )

    def handle_message(self, msg: bytes):
        """
        Handle a complete (reassembled) message
        """
        try:
            deserialized_msg = self.message_handler.decrypt_and_decode(msg)
            msg_type_name = deserialized_msg.get("msg_type_name")
            data = deserialize_message(deserialized_msg.get("data"), get_message(msg_type_name))
            topic: str = deserialized_msg.get("topic")
            hostname: str = deserialized_msg.get("hostname")
            latched: bool = deserialized_msg.get("latched")

            if hostname not in self.known_senders:
                self.known_senders.append(hostname)

            self.publish(topic, data, hostname, latched)
        except Exception as e:
            self.node.get_logger().error(f"Could not deserialize received message with error {e}")

    def publish(self, topic: str, msg, hostname: str, latched: bool):
        """
        Publish a message into ROS

        :param topic: The topic on which the message was sent on the originating host
        :param msg: The ROS message which was sent on the originating host
        :param hostname: The hostname of the originating host

        todo: this does not preserve the original qos profile
        """
        if self.params["hostname"] in topic:
            # remove everything up to and including the hostname from the topic name
            namespaced_topic = topic.split(self.params["hostname"], 1)[1]
        else:
            # publish msg under host namespace
            namespaced_topic = hostname.replace("-", "_") + topic

        # create a publisher object if we don't have one already
        if namespaced_topic not in self.publishers.keys():
            self.node.get_logger().info(f"Publishing new topic {namespaced_topic}")
            self.publishers[namespaced_topic] = self.node.create_publisher(
                type(msg),
                namespaced_topic,
                qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else 10,
            )

        self.publishers[namespaced_topic].publish(msg)

def run_spin_in_thread(node):
    # Necessary in ROS 2, or else we get stuck
    thread = Thread(target=rclpy.spin, args=[node], daemon=True)
    thread.start()


def main():
    rclpy.init()
    node = Node("udp_bridge_receiver")
    # setup udp receiver
    receiver = UdpBridgeReceiver(node)
    run_spin_in_thread(node)
    receiver.recv_message()

if __name__ == "__main__":
    main()