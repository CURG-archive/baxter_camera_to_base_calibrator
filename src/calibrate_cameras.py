#!/usr/bin/env python
import rospy
import sensor_msgs.msg
#import tf
import tf_conversions
import cv_bridge
import cv
import pose_msgs
import pose_msgs.srv
import std_srvs.srv._Empty
import time
import tf; from tf import *
from tf.transformations import *




class RobotToCheckerboardPublisher(object):
    def __init__(self):
        self.left_arm_tf_updater = CheckerboardTransformUpdater("left")
        self.right_arm_tf_updater = CheckerboardTransformUpdater("right")
        self.tf_broadcaster = TransformBroadcaster()
  
    def publish_transform(self):
        transform = self.get_most_recent_transform()
        if(transform):
            self.tf_broadcaster.sendTransform(self.xyz_, tf.transformations.quaternion_from_euler(self.rpy_[0], self.rpy_[1], self.rpy_[2]),
                     rospy.Time.now(), self.child_, self.parent_)

    def get_most_recent_transform(self):

        left_pose, left_time = self.left_arm_tf_updater.current_pose_and_time
        right_pose, right_time = self.left_arm_tf_updater.current_pose_and_time

        if(right_time > left_time):
            pose = right_pose
            camera_transform = self.right_arm_tf_updater.camera_transform
        else:
            pose = left_pose
            camera_transform = self.left_arm_tf_updater.camera_transform
       
        transform = None
        if(pose):
            checkerboard_in_camera = tf_conversions.toMatrix((tf_conversions.fromMsg(pose)))
            checkerboard_in_body = camera_transform * checkerboard_in_camera
            checkerboard_in_body_tf = tf_conversions.toTf(tf_conversions.toMatrix(checkerboard_in_body))
            checkerboard_rpy = tf.transformations.quaternion_to_euler(*checkerboard_in_body_tf[1])
            transform = checkboard_body_in_tf[0] + checkerboard_rpy

        return transform



#Create two services that when they recieve a command send a command to the checkerboard detectors 
#for their respective arm and then update the static pose from base to the world
class CheckerboardTransformUpdater(object):
    def __init__(self, arm="right"):
        self.current_pose_and_time = (None, time.time())
        self.arm = arm

        self.camera_transform = None
        self.service_server = rospy.Service('%s_arm_calibrate_base'%(self.arm), std_srvs.srv.Empty, self.service_callback)
        try:
            rospy.wait_for_service('/%s_checkerboard_pose/get_pose'%(self.arm))
            self.checkerboard_get_pose = rospy.ServiceProxy('/%s_checkerboard_pose/get_pose'%(self.arm), pose_msgs.srv.GetPose())
        except Exception as e:
           print "error"
           exit(-1)


    def service_callback(self, service_request):
       pose_response = self.checkerboard_get_pose()
       if(pose_response.success):
           self.current_pose_and_time = (pose_response.pose, time.time())


    def get_camera_tf(self):

        camera_frame_id = '/%s_camera_frame_id'%(self.arm)
        tf_listener.waitForTransform('/base',camera_frame_id, rospy.Time(0), rospy.Duration(2))
        image_tf = tf_listener.lookupTransform('/base',image.header.frame_id, rospy.Time(0))
        self.camera_transform = tf_conversions.toMatrix((tf_conversions.fromTf(image_tf)))



if __name__ == "__main__":
    rospy.init_node("checkerboard_transform_updater")

    checkerboard_tf_publisher = RobotToCheckerboardPublisher()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        checkerboard_tf_publisher.publish_transform()
        rospy.spin()




    # def update_base_pose(self, pose):
    #     camera_frame_id = '/%s_camera_frame_id'%(self.arm)

    #     tf_listener.waitForTransform('/base',camera_frame_id, rospy.Time(0), rospy.Duration(2))
    #     image_tf = tf_listener.lookupTransform('/base',image.header.frame_id, rospy.Time(0))
    #     camera_transform = tf_conversions.toMatrix((tf_conversions.fromTf(image_tf)))
    #     checkerboard_in_camera = tf_conversions.toMatrix((tf_conversions.fromMsg(pose)))
    #     checkerboard_in_body = camera_transform * checkerboard_in_camera
    #     checkerboard_in_body_tf = tf_conversions.toTf(tf_conversions.toMatrix(checkerboard_in_body))
    #     checkerboard_rpy = tf.transformations.quaternion_to_euler(*checkerboard_in_body_tf[1])

    #     static_update_request = update_transform(checkboard_body_in_tf[0] + checkerboard_rpy)
    #     self.semi_static_transform_client(static_update_request)

    # def init_node():
#     #if the node is not already initialized, initialize it
#     if rospy.get_name() =='/unnamed':
#         rospy.init_node('calibrate_cameras')
#     global tf_listener
#     if not tf_listener:
#         tf_listener = tf.TransformListener()


# def get_image_and_transform():
#     init_node()
#     global tf_listener
#     image = rospy.wait_for_message('/cameras/right_hand_camera/image',sensor_msgs.msg.Image)
#     tf_listener.waitForTransform('/base',image.header.frame_id, rospy.Time(0), rospy.Duration(2))
#     image_tf = tf_listener.lookupTransform('/base',image.header.frame_id, rospy.Time(0))
#     print image_tf
#     image_transform = tf_conversions.toMatrix((tf_conversions.fromTf(image_tf)))
    
#     return image, image_transform


# def save_image_and_transform():
#     bridge = cv_bridge.CvBridge()
#     image, transform = get_image_and_transform()
#     cv_image = bridge.imgmsg_to_cv(image, "bgr8")
#     transform_file = open('transform%d.txt'%(image.header.stamp.secs),'w')
#     transform_file.write('%s'%(transform))
#     cv.SaveImage('image%d.png'%(image.header.stamp.secs),cv_image)