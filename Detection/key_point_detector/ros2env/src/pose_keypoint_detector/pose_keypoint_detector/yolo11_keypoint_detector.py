#!/usr/bin/env python3
from cv_bridge import CvBridge
from std_msgs.msg import Header 
from ultralytics import YOLO 
from rclpy.node import Node 
import rclpy
from pose_detection_interfaces.msg import Pose2D, Pose2DKeypoint
from .pose_keypoint_detector import PoseKeypointDetector
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from .submodules.camera_subscriber import CameraSubscriber
import tf2_ros
import tf2_geometry_msgs
import cv2
from sensor_msgs.msg import Image as ROS_Image

YOLO11_KEYPOINT_DICT = { 0:'nose', 1:'l_eye', 2:'r_eye', 3:'l_ear',
                         4:'r_ear', 5:'l_sho', 6:'r_sho', 7:'l_elb', 8:'r_elb', 
                         9:'l_wri', 10:'r_wri', 11:'l_hip', 12:'r_hip',
                         13:'l_knee', 14:'r_knee', 15:'l_ank', 16:'r_ank'}

DEFAULT_FILTER = { 9 : 'l_wri', 10: 'r_wri' }

class Yolo11KeypointDetector(PoseKeypointDetector): 
    """
    Class for using YOLO-11 model to detect body keypoints from RGB stream.   

    Args:
        PoseKeypointDetector: abstracted super class for pose keypoint detection
    """
    def __init__(self, node, rgb_topic, rgb_frame,
                 filtered_keypoints=DEFAULT_FILTER):

        super().__init__(node, rgb_topic, rgb_frame)
        self._model = YOLO("yolo11n-pose.pt")
        self._bridge = CvBridge()
        
        self._filtered_keypoints = filtered_keypoints
        self._keypoint_pub = self._node.create_publisher(Pose2D, 
                                                         "pose_keypoints", 1)
        # Initialize Pose message for publisher
        self._msg = Pose2D()
        self._msg.pose_id = 0
        self._msg.header = Header() 
        self._msg.header.frame_id = rgb_frame

        self._r_wrist_poses_pub = self._node.create_publisher(PoseStamped, 
                                                         "r_wrist_poses", 10)
        self._l_wrist_poses_pub = self._node.create_publisher(PoseStamped, 
                                                         "l_wrist_poses", 10)
        self._tf_buffer = tf2_ros.Buffer() 
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        
        output_image_topic = "/wrist_detection_visualized"
        self.image_publisher = self._node.create_publisher(ROS_Image, 
                                                         output_image_topic, 10)
        self.marker_pub_r = self._node.create_publisher(Marker, 'visualization_marker_r', 10)
        
        self.marker_pub_l = self._node.create_publisher(Marker, 'visualization_marker_l', 10)

        self.camera_tf_frame = "st_cam_color_optical_frame"
        self.robot_tf_frame = "fr3_link0"
        self._conf_threshold = 0.9

        self._camera_sub = CameraSubscriber(self._node, 
                            "/camera/st_cam/color/image_raw",
                            "/camera/st_cam/aligned_depth_to_color/image_raw", 
                            "/camera/st_cam/aligned_depth_to_color/camera_info")
    
    def image_callback(self, msg): 
        """
        Upon new RGB stream msg, detect body keypoints using YOLO-11

        Args:
            msg (ROS_Image): RGB frame 
        """
        cv_image = self._bridge.imgmsg_to_cv2(msg, msg.encoding)

        # make prediction     
        keypoints = self._model.predict(source=cv_image, verbose=False)[0] \
                      .cpu().keypoints
        keypoints_xy = keypoints.xy
        confidences = keypoints.conf

        if keypoints_xy is not None and confidences is not None \
           and len(keypoints) != 0: 
            # include confidence scores  
            keypoints = [[x, y, conf] for ([x, y], conf) in 
                         zip(keypoints_xy[0].tolist(), confidences[0].tolist())]
            self.set_keypoints(keypoints)
            self.visualize_keypoints(cv_image)
            self.publish_poses()

    def publish_poses(self): 
        """
        Publish the filtered keypoints 
        """
        poses = []
        # collect the desired keypoints into keypoint list 
        for key, value in self._filtered_keypoints.items():
            x, y, conf = self.get_keypoint_by_index(key)
            pose = Pose2DKeypoint(
                kpt_name=value,
                conf=float(conf),
                x=int(x),
                y=int(y)
            )
            poses.append(pose)

        # update timestamp, publish the message
        self._msg.header.stamp = self._node.get_clock().now().to_msg()
        self._msg.keypoint_list = poses
        self._keypoint_pub.publish(self._msg)

        # fetch 3D coordinates
        for pose in poses:
            if pose.conf >= self._conf_threshold:
                if pose.kpt_name == "l_wri":
                    l_wrist_xyz = self._camera_sub.deproject_pixel_to_point([pose.x, pose.y])
                    print("left wrist xyz here: ", l_wrist_xyz)
                    l_wrist_pose = PoseStamped()
                    l_wrist_pose.header.frame_id = self.camera_tf_frame
                    l_wrist_pose.header.stamp = self._node.get_clock().now().to_msg()
                    l_wrist_pose.pose.position.x = l_wrist_xyz[0]/1000
                    l_wrist_pose.pose.position.y = l_wrist_xyz[1]/1000
                    l_wrist_pose.pose.position.z = l_wrist_xyz[2]/1000          
                    
                    l_wrist_transformed_pose = self._tf_buffer.transform(l_wrist_pose, self.robot_tf_frame, timeout=rclpy.duration.Duration(seconds=1.0))
                    self._l_wrist_poses_pub.publish(l_wrist_transformed_pose)
                    self.visualize_marker(l_wrist_transformed_pose, wrist="left")
                elif pose.kpt_name == "r_wri":
                    r_wrist_xyz = self._camera_sub.deproject_pixel_to_point([pose.x, pose.y])
                    print("right wrist xyz here: ", r_wrist_xyz)
                    r_wrist_pose = PoseStamped()
                    r_wrist_pose.header.frame_id = self.camera_tf_frame
                    r_wrist_pose.header.stamp = self._node.get_clock().now().to_msg()
                    r_wrist_pose.pose.position.x = r_wrist_xyz[0]/1000
                    r_wrist_pose.pose.position.y = r_wrist_xyz[1]/1000
                    r_wrist_pose.pose.position.z = r_wrist_xyz[2]/1000
                    
                    r_wrist_transformed_pose = self._tf_buffer.transform(r_wrist_pose, self.robot_tf_frame, timeout=rclpy.duration.Duration(seconds=1.0))
                    self._r_wrist_poses_pub.publish(r_wrist_transformed_pose)
                else:
                    #Shouldn't really end up here but:
                    print("Unknown keypoint!")

    def visualize_keypoints(self, cv_image):
    	width = cv_image.shape
    	green_color = (0, 255, 0)
    	black_color = (0, 0, 0)
    	white_color= (255, 255, 255)
    	red_color = (0, 0, 255)
    	thickness = 5
    	fontScale = 1.5
    	font = cv2.FONT_HERSHEY_DUPLEX
    	
    	for key, value in self._filtered_keypoints.items():
    		x, y, conf = self.get_keypoint_by_index(key)
    		if value == "l_wri":
    			cv2.circle(cv_image,(int(x),int(y)), 15, red_color, -1)
    			cv2.putText(cv_image, 'handLEFT', (int(x),int(y)), font, fontScale, green_color, thickness, cv2.LINE_AA)
    		elif value == "r_wri":
    			cv2.circle(cv_image,(int(x),int(y)), 15, red_color, -1)
    			cv2.putText(cv_image, 'handRIGHT', (int(x),int(y)), font, fontScale, green_color, thickness, cv2.LINE_AA)
    		else:
    			#Shouldn't really end up here but:
    			print("Unknnown keypoint!")
    			
    	self.image_publisher.publish(self._bridge.cv2_to_imgmsg(cv_image, encoding="rgb8"))
    	
    def visualize_marker(self, pose_stamped: PoseStamped, marker_id: int = 0, wrist="right"):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = "pose_visualization"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.x = 0.05
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        if wrist == "left":
            self.marker_pub_l.publish(marker)
        else:
            self.marker_pub_r.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    pose_sub = Yolo11KeypointDetector(Node("pose_subscriber"), 
                                      "/camera/st_cam/color/image_raw",
                                      "st_cam_color_optical_frame")
    rclpy.spin(pose_sub._node)
    # Destroy the node explicitly
    pose_sub._node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    