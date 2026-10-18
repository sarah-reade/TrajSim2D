""


class DirectTrajectoryTestNode(Node):
    


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveitLaunchServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
