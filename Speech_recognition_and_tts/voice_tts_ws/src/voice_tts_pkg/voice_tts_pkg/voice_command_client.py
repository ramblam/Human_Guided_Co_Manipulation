import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcm_msgs.action import VoiceCommand
from queue import Queue

class VoiceCommandClient(Node):

    def __init__(self):
        super().__init__('voice_command_client')
        self._action_client = ActionClient(self, VoiceCommand, '/rcm/voice_command')
        self.command_queue = Queue()
        self.create_timer(0.1, self._process_queue)
        
    def _process_queue(self):
        """Check if MQTT has enqueued any voice commands."""
        if not self.command_queue.empty():
            cmd = self.command_queue.get()
            self.get_logger().info(f"Processing command from MQTT: {cmd}")
            self.send_voice_command(cmd)

    def send_voice_command(self, command):
        goal_msg = VoiceCommand.Goal()
        goal_msg.voice_command = command
        
        self.get_logger().info(f"Sending command to action server: {command}")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.state))

def main(args=None):
    rclpy.init(args=args)
    action_client = VoiceCommandClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
