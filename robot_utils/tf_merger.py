import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

import yaml

class TfMerger(Node):
    def __init__(self):
        super().__init__('tf_merger')
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").value
        with open(config_path) as c:
            self.config = yaml.safe_load(c)

        self.received_tfs = {}
        self.received_static_tfs = {}
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.subscribers = []
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        for tree in self.config["trees"]:
            ns = tree["namespace"]
            self.received_tfs[f'{ns}/tf'] = []
            self.received_static_tfs[f'{ns}/tf_static'] = []
            tf_sub = self.create_subscription(TFMessage, f'{ns}/tf', lambda msg, t=f'{ns}/tf': self.tf_callback(msg, t), 10)
            tf_static_sub = self.create_subscription(TFMessage, f'{ns}/tf_static', lambda msg, t=f'{ns}/tf_static': self.tf_static_callback(msg, t), qos)
            self.subscribers.extend([tf_sub, tf_static_sub])

        self.timer = self.create_timer(0.05, self.publish_merged_tf)

    def tf_callback(self, msg, t):
        for tf in msg.transforms:
            existing = [x for x in self.received_tfs[t] if x.child_frame_id == tf.child_frame_id]
            if existing:
                self.received_tfs[t].remove(existing[0])
            self.received_tfs[t].append(tf)

    def tf_static_callback(self, msg, t):
        updated = False
        for tf in msg.transforms:
            existing = [x for x in self.received_static_tfs[t] if x.child_frame_id == tf.child_frame_id]
            if existing:
                self.received_static_tfs[t].remove(existing[0])
            self.received_static_tfs[t].append(tf)
            updated = True

        if updated:
            for tf in self.received_static_tfs[t]:
                tf_copy = TransformStamped()
                tf_copy.header.stamp = rclpy.time.Time().to_msg() # 0
                tf_copy.header.frame_id = tf.header.frame_id
                tf_copy.child_frame_id = tf.child_frame_id
                tf_copy.transform = tf.transform
                self.static_broadcaster.sendTransform(tf_copy)

    def publish_merged_tf(self):
        now = self.get_clock().now().to_msg()
        for tree in self.config["trees"]:
            ns = tree["namespace"]
            odom= f"{ns}/odom"

            t_world_odom = TransformStamped()
            t_world_odom.header.stamp = now
            t_world_odom.header.frame_id = "world"
            t_world_odom.child_frame_id = odom
            t_world_odom.transform.translation.x = tree["position"]["x"]
            t_world_odom.transform.translation.y = tree["position"]["y"]
            t_world_odom.transform.translation.z = tree["position"]["z"]
            t_world_odom.transform.rotation.x = tree["orientation"]["x"]
            t_world_odom.transform.rotation.y = tree["orientation"]["y"]
            t_world_odom.transform.rotation.z = tree["orientation"]["z"]
            t_world_odom.transform.rotation.w = tree["orientation"]["w"]
            self.tf_broadcaster.sendTransform(t_world_odom)

            for tf in self.received_tfs.get(f"{ns}/tf", []):
                if tf.child_frame_id == odom:
                    continue
                tf_copy = TransformStamped()
                tf_copy.header.stamp = now
                tf_copy.header.frame_id = tf.header.frame_id
                tf_copy.child_frame_id = tf.child_frame_id
                tf_copy.transform = tf.transform
                self.tf_broadcaster.sendTransform(tf_copy)


def main(args=None):
    rclpy.init(args=args)
    node = TfMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()