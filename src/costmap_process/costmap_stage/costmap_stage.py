# flake8: noqa
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Int32

import tf2_ros
from tf2_ros import TransformException


class StageMonitorNode(Node):
	"""Monitor map->camera_link transform and publish stage state."""

	def __init__(self) -> None:
		super().__init__('stage_monitor')

		self.declare_parameter('target_frame', 'map')
		self.declare_parameter('source_frame', 'camera_link')
		self.declare_parameter('publish_topic', 'stage')
		self.declare_parameter('lookup_timeout', 0.5)
		self.declare_parameter('timer_period', 0.1)

		self._target_frame: str = self.get_parameter('target_frame').get_parameter_value().string_value
		self._source_frame: str = self.get_parameter('source_frame').get_parameter_value().string_value
		self._publish_topic: str = self.get_parameter('publish_topic').get_parameter_value().string_value
		self._lookup_timeout: float = self.get_parameter('lookup_timeout').get_parameter_value().double_value
		self._timer_period: float = self.get_parameter('timer_period').get_parameter_value().double_value

		self._tf_buffer = tf2_ros.Buffer()
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

		# state variables
		# current stage (can be 0,1,2,3,4); initialized to None until first transform is read
		self._stage = None  # type: int | None
		# latest requested next_stage from the /next_stage topic (int) or None
		self._next_stage = None  # type: int | None

		self._publisher = self.create_publisher(Int32, self._publish_topic, 10)
		# subscribe to /next_stage topic to receive requested stage transitions
		self._next_stage_sub = self.create_subscription(Int32, '/next_stage', self._on_next_stage, 10)
		self._timer = self.create_timer(self._timer_period, self._on_timer)

		self._stage_region = self._make_stage_region((3.98, 1.04), (5.0, 0.55))

		self.get_logger().info(
			f"Stage monitor running (target={self._target_frame}, source={self._source_frame}, "
			f"topic={self._publish_topic})"
		)

	@staticmethod
	def _make_stage_region(corner_a: Tuple[float, float], corner_b: Tuple[float, float]) -> Tuple[float, float, float, float]:
		x_min, x_max = sorted((corner_a[0], corner_b[0]))
		y_min, y_max = sorted((corner_a[1], corner_b[1]))
		return x_min, x_max, y_min, y_max

	def _compute_stage(self, x: float, y: float) -> int:
		x_min, x_max, y_min, y_max = self._stage_region
		in_region = x_min <= x <= x_max and y_min <= y <= y_max
		return 0 if in_region else 1

	def _on_next_stage(self, msg: Int32) -> None:
		"""Callback for /next_stage subscription. Stores the requested next stage."""
		try:
			next_val = int(msg.data)
		except Exception:
			self.get_logger().warn(f"Received non-int next_stage: {msg.data}")
			return

		self._next_stage = next_val
		self.get_logger().info(f"Received /next_stage={self._next_stage}")

	def _on_timer(self) -> None:
		try:
			transform = self._tf_buffer.lookup_transform(
				self._target_frame,
				self._source_frame,
				Time(),
				timeout=Duration(seconds=self._lookup_timeout),
			)
		except TransformException as exc:
			self.get_logger().warn(
				f"Failed to lookup transform {self._target_frame} -> {self._source_frame}: {exc}",
				throttle_duration_sec=5.0,
			)
			return

		translation = transform.transform.translation
		x, y = translation.x, translation.y

		# compute base stage from transform (0 if inside region, 1 otherwise)
		base_stage = self._compute_stage(x, y)

		# initialize stage on first run
		if self._stage is None:
			self._stage = base_stage
			self.get_logger().info(f"Initial stage set to {self._stage} based on position")

		# apply next_stage transition logic if a request exists
		if self._next_stage is not None:
			next_val = self._next_stage
			applied = False
			# If current stage == 0, allow transitions to 1 or 2
			if self._stage == 0 and next_val in (1, 2):
				self._stage = next_val
				applied = True
			# If current stage == 1 or 2, allow transitions to 3 or 4
			elif self._stage in (1, 2) and next_val in (3, 4):
				self._stage = next_val
				applied = True

			if applied:
				self.get_logger().info(f"Applied /next_stage -> stage now {self._stage}")
				# consume the request so it's not applied repeatedly
				self._next_stage = None
			else:
				self.get_logger().info(f"Ignored /next_stage={next_val} for current stage={self._stage}")

		# If there is no explicit requested transition, keep using the current stage.
		# Optionally, you could also update based on base_stage here if desired.

		msg = Int32()
		msg.data = int(self._stage)
		self._publisher.publish(msg)

		self.get_logger().info(
			f"Transform {self._target_frame}->{self._source_frame}: x={x:.3f}, y={y:.3f}, z={translation.z:.3f}, "
			f"base={base_stage}, stage={self._stage}",
			throttle_duration_sec=1.0,
		)


def main(args=None) -> None:
	rclpy.init(args=args)
	node = StageMonitorNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

