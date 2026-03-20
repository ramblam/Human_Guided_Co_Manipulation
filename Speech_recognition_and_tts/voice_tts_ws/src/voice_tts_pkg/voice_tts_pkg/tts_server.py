import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rcm_msgs.action import TtsCommand
import json

class TtsActionServer(Node):

    def __init__(self, mqtt_client):
        super().__init__('tts_action_server')
        self._action_server = ActionServer(
            self,
            TtsCommand,
            '/rcm/tts_msg',
            self.execute_callback)
        self.mqtt_client = mqtt_client

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Handling TTS-message...')

        feedback_msg = TtsCommand.Feedback()
        feedback_msg.state = "Handling text-to-speech command..."
        
        msg = goal_handle.request.tts_command
        if msg =="setContextMain":
            #This is used for extending the timeout if it runs out during robot actions.
            formatPayload = {"contextName":'MAIN_OVS', "timeOut":10000}
            json_dump = json.dumps(formatPayload)
            self.mqtt_client.publish("creoir/asr/setContext", json_dump)
        else:
            formatPayload = {"utterance":msg}
            json_dump = json.dumps(formatPayload)
            self.mqtt_client.publish("creoir/talk/speak", json_dump)
        goal_handle.succeed()
        
        return TtsCommand.Result(success=True)
