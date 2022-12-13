#!/usr/bin/env python3
import apriltag
import cv2
import rospy
# from keras.model import load_model
# from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Bool


class StopOrStraight:
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # Set the maximum update rate for commands
        self.rate = rospy.Rate(30)  # 30hz
        self.labels = [0, 1, 2]
        # self.model = load_model('keras_model.h5')
        '''
        options = apriltag.Detectoroptions(families='tag36h11',
                                            border=1,
                                            nthreads=4,
                                            quad_decimate=1.0,
                                            quad_blur=0.0,
                                            refine_edges=True,
                                            refine_decode=False,
                                            refine_pose=False,
                                            debug=False,
                                            quad_contours=True)
        '''
        self.detector = apriltag.Detector()
        # Set up publishers and subscribers
        self.stop = None
        self.pub_wheels_cmd = rospy.Publisher("/duckie007/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        # self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.sub_stop = rospy.Subscriber("~stop_sign", Bool, self.cbStop, queue_size=1) # only process the latest
        rospy.loginfo(f"[{self.node_name}] Initialzed.")

    def processImage(self, image_msg):
        # get an openCV version of the image
        np_arr = np.fromstring(image_msg.data, np.uint8)
        image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
    
        # TODO CV goes here!
        result = self.detector.detect(image_cv)
        if len(result) > 0:
            tag_id = result[0].tag_id
            if tag_id == 1:
                corners = result[0].corners
            self.pub_sign.publish(Bool(data=True))
        else:
            self.pub_sign.publish(Bool(data=False))
        
        
        '''
        im = cv2.resize(image_cv, (224,224), interpolation=cv2.INTER_AREA)
        im = np.asarray(im, dtype=np.float32).reshape(1, 224, 224, 3)
        im = (im / 127.5) - 1
        prob = model.predict(im)
        label = labels[np.argmax(prob)]
        if label == 0:
            self.pub_sign.publish(Bool(data=True))
        else: # publish no sign (you might want to update this)
            self.pub_sign.publish(Bool(data=False))
        '''
    # save down the latest data on the stop_sign channel
    def cbStop(self, data):
        self.stop = data

    def main(self):
        while not rospy.is_shutdown():
            # go forward unless we see a stop sign
            if self.stop:
                vel = 0
            else:
                vel = 0.2

            # make the object the duckiebot is looking for
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = vel
            wheels_cmd_msg.vel_right = vel
            self.pub_wheels_cmd.publish(wheels_cmd_msg)

            # sleep to not send commands too often
            self.rate.sleep()

if __name__ == "__main__":
    # Initialize the node with rospy
    rospy.init_node("stop_or_straight_node", anonymous=False)

    # Create the object
    node = StopOrStraight()
    node.main()
