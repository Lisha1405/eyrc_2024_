import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from linkattacher_msgs.srv import AttachLink, DetachLink

class UR5Mover(Node):
    def __init__(self):
        super().__init__('ur5_mover')
        
        # Publisher for the arm pose
        self.arm_pub = self.create_publisher(PoseStamped, '/ur5/arm_pose', 10)
        
        # Create clients for gripper services
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')
        
        # Wait for services to be available
        self.wait_for_service(self.attach_client)
        self.wait_for_service(self.detach_client)

        # Move sequence
        self.move_sequence()

    def wait_for_service(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def move_sequence(self):
        # Define points
        points = [
            [0.20, -0.47, 0.65],  # P1
            [-0.69, 0.10, 0.44],  # D
            [0.75, 0.49, -0.05],  # P2
            [-0.69, 0.10, 0.44],  # D
            [0.75, -0.23, -0.05],  # P3
            [-0.69, 0.10, 0.44]   # D
        ]

        for point in points:
            self.move_to_position(point)

    def move_to_position(self, position):
        # Create the PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        
        # Publish the pose
        self.arm_pub.publish(pose_msg)
        self.get_logger().info(f'Moving to position: {position}')

        # Wait for some time for the movement to complete
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))

        # Check if it's a 'D' position to toggle the gripper
        if position == [-0.69, 0.10, 0.44]:
            self.toggle_gripper()

    def toggle_gripper(self):
        # Attach the box
        self.call_gripper_service(self.attach_client, 'box1')

        # Detach the box
        self.call_gripper_service(self.detach_client, 'box1')

    def call_gripper_service(self, client, box_name):
        request = AttachLink.Request()
        request.model1_name = box_name
        request.link1_name = 'link'
        request.model2_name = 'ur5'
        request.link2_name = 'wrist_3_link'
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Gripper service call success: {future.result()}')
        else:
            self.get_logger().error('Gripper service call failed')

def main(args=None):
    rclpy.init(args=args)
    ur5_mover = UR5Mover()
    rclpy.spin(ur5_mover)
    ur5_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
