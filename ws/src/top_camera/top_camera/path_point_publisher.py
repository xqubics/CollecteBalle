import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from terrain_balls import TerrainBalls


class PathPointPublisher(Node):
    """
        Path point publisher - publishes a path point to `robot/target_position` topic
        every 0.5 seconds
    """

    def __init__(self):
        super().__init__('path_point_publisher')
        self._timer = self.create_timer(0.5, self._timer_callback)
        self._publisher = self.create_publisher(Pose2D, 'robot/target_position', 10)
        self._terrain_balls = TerrainBalls(0, display_camera=True)

    def _timer_callback(self):

        target_pos = self._terrain_balls.get_latest_path_point()

        if target_pos is None:
            return

        msg = Pose2D()
        msg.x = float(target_pos[0])
        msg.y = float(target_pos[1])
        msg.theta = 0.  # only required position can be specified for now, not heading

        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    path_point_publisher = PathPointPublisher()

    try:
        rclpy.spin(path_point_publisher)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    path_point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
