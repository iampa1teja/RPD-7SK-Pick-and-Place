import rclpy
from rclpy.node import Node
from moveit_7sk.srv import GoToNamedPose, GoToPose, SetGripper


class MoveBotClient(Node):
    def __init__(self):
        super().__init__('move_bot_client')

        self.named_pose_client = self.create_client(GoToNamedPose, 'go_to_named_pose')
        self.pose_client = self.create_client(GoToPose, 'go_to_pose')
        self.gripper_client = self.create_client(SetGripper, 'set_gripper')

        # wait for all services to be ready
        self.named_pose_client.wait_for_service()
        self.pose_client.wait_for_service()
        self.gripper_client.wait_for_service()
        self.get_logger().info('All services ready')

    def _call_service(self, client, request, action_name: str) -> bool:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error(f'{action_name} failed: no response from service')
            return False

        if not result.success:
            message = result.message if hasattr(result, 'message') else ''
            if message:
                self.get_logger().error(f'{action_name} failed: {message}')
            else:
                self.get_logger().error(f'{action_name} failed')
            return False

        message = result.message if hasattr(result, 'message') else ''
        if message:
            self.get_logger().info(f'{action_name} succeeded: {message}')
        return True

    def go_to_named_pose(self, name: str) -> bool:
        req = GoToNamedPose.Request()
        req.pose_name = name
        return self._call_service(self.named_pose_client, req, f'go_to_named_pose({name})')

    def go_to_pose(self, x, y, z, r=-1.57, p=0.0, yaw=0.0) -> bool:
        req = GoToPose.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        req.roll, req.pitch, req.yaw = float(r), float(p), float(yaw)
        return self._call_service(self.pose_client, req, f'go_to_pose({x}, {y}, {z})')

    def set_gripper(self, value: float) -> bool:
        req = SetGripper.Request()
        req.value = float(value)
        return self._call_service(self.gripper_client, req, f'set_gripper({value})')