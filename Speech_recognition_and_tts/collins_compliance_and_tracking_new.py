import paho.mqtt.client as mqtt
import json
import socket
from datetime import datetime
import threading
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcm_msgs.action import VoiceCommand
from rclpy.action import ActionClient
from voice_tts_pkg.voice_command_client import VoiceCommandClient
from voice_tts_pkg.tts_server import TtsActionServer
from rclpy.executors import MultiThreadedExecutor

audioPath = "/usr/share/creoir/"
lastAction = ""
sub_listen = True
wakeup_counter = 0
task = 1

#Boilerplate connect callback function.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("creoir/#", 0)

#Callback funtion to "creoir/asr/intentRecognized" topic.
def on_message_intent(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    collins_proactivity_eventloop(msg.payload, msg.topic, userdata)
    #This avoids going back to WUW:
    formatPayload = {"contextName":'MAIN_OVS', "timeOut":10000}
    json_dump = json.dumps(formatPayload)
    client.publish("creoir/asr/setContext", json_dump)

#Callback function to "creoir/asr/listening" topic.
def on_message_wuw(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    collins_proactivity_eventloop(msg.payload, msg.topic, userdata)

#Callback function to "creoir/asr/intentNotRecognized" topic.
def on_message_timeout(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    sub_listen = False
    collins_proactivity_eventloop(msg.payload, msg.topic, userdata)

#Callback function to "creoir/device/versioninfo" topic.
def on_message_version(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    collins_proactivity_eventloop(msg.payload, msg.topic, userdata)

#Callback function to "creoir/config/current" topic.
def on_message_config(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))

#MQTT handler function. This function in target for the thread created in the main.
def mqttHandler(client):
    client.message_callback_add('creoir/device/versioninfo', on_message_version)
    client.message_callback_add('creoir/asr/intentRecognized', on_message_intent)
    client.message_callback_add('creoir/asr/listening', on_message_wuw)
    client.message_callback_add('creoir/asr/intentNotRecognized', on_message_timeout)
    client.message_callback_add('creoir/config/current', on_message_config)

    client.on_connect = on_connect
    client.connect("localhost", 1883, 60)
    client.loop_forever()

#Function used to set the OVS parameters manually.
def setOvsParameters():
    config = {}
    config["utteranceThreshold"] = 4500
    config["wakewordThreshold"] = 4200
    config["slotThreshold"] = 0
    config["sendTestEvents"] = 0
    config["silenceTimeout"] = 10000
    config["babbleTimeout"] = 0
    payload = json.dumps(config)
    client.publish("creoir/config/set", payload)

def placeObject(objectName, jsonData):
        print(objectName)
        print(jsonData)
        voice_command_client.send_voice_command("PLACE "+objectName.upper())
        
def pickObject(objectName, jsonData):
        print(objectName)
        print(jsonData)
        voice_command_client.send_voice_command("PICK "+objectName.upper())

def activateController(controllerName, jsonData):
        print(controllerName)
        print(jsonData)
        voice_command_client.send_voice_command("ACTIVATE "+controllerName.upper())
        
def setMode(modeName, jsonData):
        print(modeName)
        print(jsonData)
        voice_command_client.send_voice_command("MODE "+modeName.upper())
        
def moveCommand(direction, jsonData):
        print(direction)
        print(jsonData)
        voice_command_client.send_voice_command("MOVE "+direction.upper())
        
def setStepSize(stepSize, jsonData):
        print(stepSize)
        print(jsonData)
        voice_command_client.send_voice_command("STEP SIZE "+stepSize.upper())

#Utility function used for controlling the TTS. Inputs: String payload str payload_type which determines if the payload is a file or a text.
def TTS_SayUtteranceMQTT(payload, payload_type="text"):
    if "wav" not in payload_type:
        formatPayload = {"utterance":payload}
        json_dump = json.dumps(formatPayload)
        client.publish("creoir/talk/speak", json_dump)
    else:
        formatPayload = {"file":audioPath + payload}
        json_dump = json.dumps(formatPayload)
        client.publish("creoir/talk/speak", json_dump)

#Utility function used to simplify the eventloop and move functionality out of it.
#This function is used to change the grammar of the OVS. Takes grammar name and desired timeout as input.
def switchContext(grammar, timeout):
    formatPayload = {"contextName":grammar, "timeOut":timeout}
    json_dump = json.dumps(formatPayload)
    client.publish("creoir/asr/setContext", json_dump)

#Eventloop. This function is in charge of parsing the correct intent from the mqtt message.
#It then either takes the action or passes the message to utility function for further parsing.
#This function is called directly from the mqtt callback functions.
#Takes mqtt message and topic as an input.
def collins_proactivity_eventloop(msg, topic, userdata):
    global lastAction
    global task
    global wakeup_counter
    print("msg here: ", msg)
    print("userdata here: ", userdata)
    jsonData = json.loads(msg)
    if "intentRecognized" not in topic:
        if "listening" in topic:
            TTS_SayUtteranceMQTT("wakeup.wav", "wav")
            if wakeup_counter==0 and task == 1:
            	TTS_SayUtteranceMQTT("Hello, Let's start the co-manipulation of the ply")
            	wakeup_counter +=1
        elif "intentNotRecognized" in topic:
            TTS_SayUtteranceMQTT("low_confidence.wav", "wav")
    else:
        if "PLACE_OBJECT" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "object" in jsonData["slots"][0]["slotName"]:
                    objectName = jsonData["slots"][0]["slotValue"]
                    placeObject(objectName, jsonData)
                    return
            lastAction = jsonData
        elif "PAUSE" in jsonData["intent"]:
        	voice_command_client.send_voice_command("PAUSE")
        	lastAction = jsonData
        elif "APPROACH" in jsonData["intent"]:
        	voice_command_client.send_voice_command("APPROACH")
        	lastAction = jsonData
        elif "GIVE_INSTRUCTIONS" in jsonData["intent"]:
            if task == 1:
                formatTTS = ("I can help you with the ply")
            TTS_SayUtteranceMQTT(formatTTS)
            lastAction = jsonData
            
        elif "PICK_OBJECT" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "object" in jsonData["slots"][0]["slotName"]:
                    objectName = jsonData["slots"][0]["slotValue"]
                    pickObject(objectName, jsonData)
                    return
            lastAction = jsonData
            
        elif "QUALITY_CHECK" in jsonData["intent"]:
        	voice_command_client.send_voice_command("QUALITY CHECK")
        	lastAction = jsonData
                  
        elif "GO_HOME" in jsonData["intent"]:
        	voice_command_client.send_voice_command("MOVE HOME")
        	lastAction = jsonData

        elif "MOVE_DIST_UNIT_DIR" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "OVS_number" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "distance" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "unit" in jsonData["slots"][i]["slotName"]:
                        unit = jsonData["slots"][i]["slotValue"]
                    elif "direction" in jsonData["slots"][i]["slotName"]:
                        direction = jsonData["slots"][i]["slotValue"]
                voice_command_client.send_voice_command("MOVE "+direction.upper()+" "+distance+" "+unit.upper())

            lastAction = jsonData

        elif "SET_REF_POSE_DIST_UNIT_DIR" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "OVS_number" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "distance" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "unit" in jsonData["slots"][i]["slotName"]:
                        unit = jsonData["slots"][i]["slotValue"]
                    elif "direction" in jsonData["slots"][i]["slotName"]:
                        direction = jsonData["slots"][i]["slotValue"]
                voice_command_client.send_voice_command("SET REFERENCE POSE "+distance+" "+unit.upper()+" "+direction.upper())

            lastAction = jsonData

        elif "SET_REF_POSE_CURRENT" in jsonData["intent"]:
        	voice_command_client.send_voice_command("SET REFERENCE POSE CURRENT")
        	lastAction = jsonData
            
        elif "CONTINUE" in jsonData["intent"]:
        	voice_command_client.send_voice_command("CONTINUE")
        	lastAction = jsonData
            
        elif "TOOL_OPEN" in jsonData["intent"]:
        	voice_command_client.send_voice_command("TOOL OPEN")
        	lastAction = jsonData
        elif "TOOL_CLOSE" in jsonData["intent"]:
        	voice_command_client.send_voice_command("TOOL CLOSE")
        	lastAction = jsonData
        elif "TOOL_ROTATE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "rotation_direction" in jsonData["slots"][i]["slotName"]:
                    	rot_dir = jsonData["slots"][i]["slotValue"]
                voice_command_client.send_voice_command("TOOL ROTATE "+rot_dir.upper())
                lastAction = jsonData
        elif "SET_MODE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "mode" in jsonData["slots"][0]["slotName"]:
                    modeName = jsonData["slots"][0]["slotValue"]
                    setMode(modeName, jsonData)
                    return
            lastAction = jsonData
        elif "MOVE" in jsonData["intent"]:
        	if len(jsonData["slots"]) != 0:
        		if "direction" in jsonData["slots"][0]["slotName"]:
        			direction = jsonData["slots"][0]["slotValue"]
        			userdata.command_queue.put("MOVE "+direction.upper())
        			return
        	lastAction = jsonData
        elif "SET_STEP_SIZE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "step_size" in jsonData["slots"][0]["slotName"]:
                    step_size = jsonData["slots"][0]["slotValue"]
                    setStepSize(step_size, jsonData)
                    return
            lastAction = jsonData
        elif "ACTIVATE_CONTROLLER" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "controller" in jsonData["slots"][0]["slotName"]:
                    controllerName = jsonData["slots"][0]["slotValue"]
                    activateController(controllerName, jsonData)
                    return
            lastAction = jsonData
        elif "DEACTIVATE_WRIST_FOLLOWING_REF" in jsonData["intent"]:
        	voice_command_client.send_voice_command("DEACTIVATE WRIST FOLLOWING REF")
        	lastAction = jsonData    
        elif "ACTIVATE_WRIST_FOLLOWING_REF" in jsonData["intent"]:
        	voice_command_client.send_voice_command("ACTIVATE WRIST FOLLOWING REF")
        	lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ONLY_X" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ONLY DIRECTION X")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ONLY_Y" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ONLY DIRECTION Y")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ONLY_Z" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ONLY DIRECTION Z")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ALSO_X" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ALSO DIRECTION X")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ALSO_Y" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ALSO DIRECTION Y")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ALSO_Z" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ALSO DIRECTION Z")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_XYZ" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE XYZ")
            lastAction = jsonData
        elif "DISALLOW_COMPLIANCE_ALL" in jsonData["intent"]:
            voice_command_client.send_voice_command("DISALLOW COMPLIANCE ALL")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ONLY_X" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ONLY DIRECTION X")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ONLY_Y" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ONLY DIRECTION Y")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ONLY_Z" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ONLY DIRECTION Z")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ALSO_X" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ALSO DIRECTION X")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ALSO_Y" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ALSO DIRECTION Y")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ALSO_Z" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ALSO DIRECTION Z")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_XYZ" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE XYZ")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_ALL" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE ALL")
            lastAction = jsonData
        elif "ALLOW_COMPLIANCE_PLANE" in jsonData["intent"]:
            voice_command_client.send_voice_command("ALLOW COMPLIANCE PLANE")
            lastAction = jsonData
        elif "RESIST_MORE_ALL_CURRENT" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST MORE CURRENT")
            lastAction = jsonData
        elif "RESIST_MORE_ALL" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST MORE ALL")
            lastAction = jsonData
        elif "RESIST_MORE_X" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST MORE X")
            lastAction = jsonData
        elif "RESIST_MORE_Y" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST MORE Y")
            lastAction = jsonData
        elif "RESIST_MORE_Z" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST MORE Z")
            lastAction = jsonData
        elif "RESIST_LESS_ALL_CURRENT" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST LESS CURRENT")
            lastAction = jsonData
        elif "RESIST_LESS_ALL" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST LESS ALL")
            lastAction = jsonData
        elif "RESIST_LESS_X" in jsonData["intent"]:
            voice_command_client.send_voice_command("RESIST LESS X")
            lastAction = jsonData
        elif "RESIST_LESS_Y" in jsonData["intent"]:
            voice_comoicmand_client.send_voice_command("RESIST LESS Y")
            lastAction = jsonData
        elif "RESIST_LESS_Z" in jsonData["intent"]:
            voice_comoicmand_client.send_voice_command("RESIST LESS Z")
            lastAction = jsonData
        elif "SET_SPEED" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "OVS_number" in jsonData["slots"][i]["slotName"]:
                        speed = jsonData["slots"][i]["slotValue"]
                    elif "speed" in jsonData["slots"][i]["slotName"]:
                        speed = jsonData["slots"][i]["slotValue"]
                voice_comoicmand_client.send_voice_command("SET SPEED "+speed)
            lastAction = jsonData
        elif "GO_SLOWER" in jsonData["intent"]:
            voice_comoicmand_client.send_voice_command("DECREASE SPEED")
            lastAction = jsonData
        elif "GO_FASTER" in jsonData["intent"]:
            voice_comoicmand_client.send_voice_command("INCREASE SPEED")
            lastAction = jsonData
        elif "ROTATE_PLY" in jsonData["intent"]:
            voice_comoicmand_client.send_voice_command("ROTATE PLY")
            lastAction = jsonData
        elif "STOP_LISTENING" in jsonData["intent"]:
            voice_command_client.send_voice_command("STOP LISTENING")
            lastAction = jsonData
        elif "END_HAND_GUIDING" in jsonData["intent"]:
            voice_command_client.send_voice_command("END HAND GUIDING")
            lastAction = jsonData
        else:
            return

#The main function of the code. It initializes the MQTT client, starts the MQTT handle thread,
#Gets the current configuration and changes that configuration.
#It then enters sleep loop, giving the thread function all of the processing time.
if __name__ == "__main__":
    try:
        rclpy.init()
        voice_command_client = VoiceCommandClient()
        client = mqtt.Client(userdata=voice_command_client)
        tts_server = TtsActionServer(client)
        
        t1 = threading.Thread(target=mqttHandler, args=(client,), daemon=True)
        t1.start()

        time.sleep(1)
        client.publish("creoir/config/get", payload=None)
        time.sleep(2)
        setOvsParameters()
        client.publish("creoir/config/get", payload=None)
        time.sleep(1)
        
        # Create the multithreaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(voice_command_client)
        executor.add_node(tts_server)

        # Start executor in a separate thread
        executor.spin()

    except KeyboardInterrupt:
        print("Caught Ctrl+C, shutting down...")
        executor.shutdown()
        voice_command_client.destroy_node()
        tts_server.destroy_node()
        rclpy.shutdown()
        client.disconnect()
        client.loop_stop()

