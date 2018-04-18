#!/usr/bin/env python

import rospy
import tf
import tf.transformations as t
import cv2
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
import numpy as np
import math
from cv_bridge import CvBridge

NODE_NAME = "mocap_head_localisation"

NAOQI_HEAD_FRAME = "Head"
NAOQI_CAMERA_DEPTH_FRAME = "CameraTop_optical_frame"
CHESSBOARD_IMAGE_FRAME = "Chessboard_image_frame"
CHESSBOARD_RB_FRAME = "mocap_chessboard"
HEAD_RB_FRAME = "mocap_robot_head"
OPTITRACK_FRAME = "optitrack"
BASELINK_FRAME = "base_link"
NAOQI_BASE_FOOTPRINT_FRAME = "base_footprint"
FOOTPRINT_RB_FRAME = "base_footprint_ground_truth"

CAMERA_SUB_TOPIC = "/naoqi_driver_node/camera/front/image_rect/compressed"
CAMERA_PARAM_SUB_TOPIC = "/naoqi_driver_node/camera/front/camera_info"
#CAMERA_SUB_TOPIC = "/naoqi_driver_node/camera/front/image_raw/compressed"
#CAMERA_PARAM_SUB_TOPIC = "/naoqi_driver_node/camera/front/camera_info"
#CAMERA_SUB_TOPIC = "/usb_cam/image_raw/compressed"
#CAMERA_PARAM_SUB_TOPIC = "/usb_cam/camera_info"

chessboard_size = (9,6)
chessboard_square_size = 0.0246875

chessboard_image_2_rb_translation = (0, 0, 0)
chessboard_image_2_rb_rotation = (1,0,0,0)

class MocapHeadCalib:
    def __init__(self):

        self._image_np = None
        self._camera_matrix = None
        self._camera_distortion = None

        self._bridge = CvBridge()

        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        #self._tf_broadcaster.sendTransform(chessboard_image_2_rb_translation, chessboard_image_2_rb_rotation,
        #                                                       rospy.Time.now(), CHESSBOARD_RB_FRAME, CHESSBOARD_IMAGE_FRAME)
        self._image_sub = rospy.Subscriber(CAMERA_SUB_TOPIC, CompressedImage, self._on_new_image)
        self._cam_param_sub = rospy.Subscriber(CAMERA_PARAM_SUB_TOPIC, CameraInfo, self._on_new_cam_info)

        self._debug_im_pub = rospy.Publisher("/debug_img", Image)

        self.chessboard_model = np.zeros((chessboard_size[0] * chessboard_size[1],3), np.float32)
        self.chessboard_model[:, :2] = np.mgrid[0:chessboard_size[0]*chessboard_square_size:chessboard_square_size, 0:chessboard_size[1]*chessboard_square_size:chessboard_square_size].T.reshape(-1, 2)

        self.base_footprint_ground_truth2base_footprint_offset = None
        self.head2head_mocap_offset = None
        # self.chessboard_model = []
        # #self.chessboard_model = np.zeros((chessboard_size[0]* chessboard_size[1], 1, 3), np.float32)
        # for i in range(chessboard_size[0]):
        #     for j in range(chessboard_size[1]):
        #         self.chessboard_model.append([[i * chessboard_square_size, j * chessboard_square_size, 0.]])
        # self.chessboard_model = np.array(self.chessboard_model)
        #rospy.loginfo(self.chessboard_model)


    def _on_new_image(self, ros_data):
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #self._image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self._image_np = self._bridge.compressed_imgmsg_to_cv2(ros_data, "bgr8")

    def _on_new_cam_info(self, infos):
        self._camera_matrix = np.array(infos.P).reshape(3,4)[:, :3].reshape(3,3)
        self._camera_distortion = np.array(infos.D).reshape(5,1)

    def run(self, _):
        if self._camera_matrix is not None and self._camera_distortion is not None and self._image_np is not None:
            rospy.loginfo_throttle(1, "Going to find chessboard corners")
            #gray = cv2.cvtColor(self._image_np, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(self._image_np, chessboard_size, flags=cv2.CALIB_CB_FAST_CHECK)
            if ret:
                corners = np.flip(corners, 0)
                # rospy.loginfo_throttle(1, corners)
                # rospy.loginfo_throttle(1, corners.shape)
                # rospy.loginfo_throttle(1, self.chessboard_model.shape)
                #rospy.loginfo_throttle(1, "Chessboard corners found, going to solve pnp")

                dbg_img = self._image_np.copy()
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                cv2.drawChessboardCorners(dbg_img, chessboard_size, corners, ret)
                #corners2 = cv2.cornerSubPix(self._image_np, corners, (11, 11), (-1, -1), criteria)

                # Find the rotation and translation vectors.
                success, rvecs, tvecs, _ = cv2.solvePnPRansac(self.chessboard_model, corners, self._camera_matrix, self._camera_distortion, flags=cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess=False)
                #rospy.loginfo_throttle(1, "Rvec : {}; Tvec : {}".format(rvecs, tvecs))
                axis = np.float32([[0.1, 0, 0], [0, 0.1, 0], [0, 0, -0.1]]).reshape(-1, 3)
                imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, self._camera_matrix, self._camera_distortion)
                dbg_img = self.drawAxis(dbg_img, corners, imgpts)
                self._debug_im_pub.publish(self._bridge.cv2_to_imgmsg(dbg_img, "mono8"))
                rot_q = self.rodriguez_to_quaternion(rvecs)
                self._tf_broadcaster.sendTransform(tvecs, rot_q, rospy.Time.now(), CHESSBOARD_IMAGE_FRAME, NAOQI_CAMERA_DEPTH_FRAME)
                self.calibrate_first_loop()

    def calibrate_first_loop(self):
        self._tf_listener.waitForTransform(CHESSBOARD_RB_FRAME, FOOTPRINT_RB_FRAME, rospy.Time(0), rospy.Duration(10))
        self._tf_listener.waitForTransform(NAOQI_BASE_FOOTPRINT_FRAME, CHESSBOARD_IMAGE_FRAME, rospy.Time(0), rospy.Duration(10))
        # t_footprintmc2mocap, r_footprintmc2mocap = self._tf_listener.lookupTransform(OPTITRACK_FRAME, FOOTPRINT_RB_FRAME, rospy.Time(0))
        # t_mocap2chessboardmc, r_mocap2chessboardmc = self._tf_listener.lookupTransform(CHESSBOARD_RB_FRAME, OPTITRACK_FRAME, rospy.Time(0))
        t_footprintmc2chessboardmc, r_footprintmc2chessboardmc = self._tf_listener.lookupTransform(CHESSBOARD_RB_FRAME, FOOTPRINT_RB_FRAME, rospy.Time(0))
        t_chessboardmc2chessboardimg, r_chessboardmc2chessboardimg = chessboard_image_2_rb_translation, chessboard_image_2_rb_rotation
        # t_chessboardimg2depth, r_chessboardimg2depth = self._tf_listener.lookupTransform(NAOQI_CAMERA_DEPTH_FRAME, CHESSBOARD_IMAGE_FRAME, rospy.Time(0))
        # t_depth2basefootprint, r_depth2basefootprint = self._tf_listener.lookupTransform(NAOQI_BASE_FOOTPRINT_FRAME, NAOQI_CAMERA_DEPTH_FRAME, rospy.Time(0))
        t_chessboardimg2basefootprint, r_chessboardimg2basefootprint = self._tf_listener.lookupTransform(NAOQI_BASE_FOOTPRINT_FRAME, CHESSBOARD_IMAGE_FRAME, rospy.Time(0))

        # rospy.loginfo_throttle(1, "basefootgt -> optitrack : {}\noptitrack -> chessboard_mocap : {}\ncb_mocap -> cb_img : {}\ncb_img -> optical : {}\noptical -> basefoot : {}".format(
        #     t_footprintmc2mocap, t_mocap2chessboardmc, t_chessboardmc2chessboardimg, t_chessboardimg2depth, t_depth2basefootprint
        # ))


        # footprintmc2mocap = transformation_matrix(t_footprintmc2mocap, r_footprintmc2mocap)
        # mocap2chessboardmc = transformation_matrix(t_mocap2chessboardmc, r_mocap2chessboardmc)
        footprintmc2chessboardmc = transformation_matrix(t_footprintmc2chessboardmc, r_footprintmc2chessboardmc)
        chessboardmc2chessboardimg = transformation_matrix(t_chessboardmc2chessboardimg, r_chessboardmc2chessboardimg)
        # chessboardimg2depth = transformation_matrix(t_chessboardimg2depth, r_chessboardimg2depth)
        # depth2basefootprint = transformation_matrix(t_depth2basefootprint, r_depth2basefootprint)
        chessboardimg2basefootprint = transformation_matrix(t_chessboardimg2basefootprint, r_chessboardimg2basefootprint)

        # footprintmc2mocap = t.compose_matrix(angles=t.euler_from_quaternion(r_footprintmc2mocap), translate=t_footprintmc2mocap)
        # mocap2chessboardmc = t.compose_matrix(angles=t.euler_from_quaternion(r_mocap2chessboardmc), translate=t_mocap2chessboardmc)
        # chessboardmc2chessboardimg = t.compose_matrix(angles=t.euler_from_quaternion(r_chessboardmc2chessboardimg), translate=t_chessboardmc2chessboardimg)
        # chessboardimg2depth = t.compose_matrix(angles=t.euler_from_quaternion(r_chessboardimg2depth), translate=t_chessboardimg2depth)
        # depth2basefootprint = t.compose_matrix(angles=t.euler_from_quaternion(r_depth2basefootprint), translate=t_depth2basefootprint)

        self.base_footprint_ground_truth2base_footprint_offset = np.dot(np.dot(footprintmc2chessboardmc, chessboardmc2chessboardimg), chessboardimg2basefootprint)

        rospy.loginfo_throttle(1,"Base footprint -> Base footprint ground truth offset : rotation : {}, translation : {}".format(*(t.decompose_matrix(self.base_footprint_ground_truth2base_footprint_offset)[2:4])))

        self.calibrate_second_loop()

    def calibrate_second_loop(self):
        t_head2base_footprint, r_head2base_footprint = self._tf_listener.lookupTransform(NAOQI_BASE_FOOTPRINT_FRAME, NAOQI_HEAD_FRAME, rospy.Time(0))
        t_base_footprint_gt2head_mocap, r_base_footprint_gt2head_mocap = self._tf_listener.lookupTransform(HEAD_RB_FRAME, FOOTPRINT_RB_FRAME, rospy.Time(0))

        head2base_footprint = t.compose_matrix(angles=t.euler_from_quaternion(r_head2base_footprint), translate=t_head2base_footprint)
        base_footprint2base_footprintgt = t.inverse_matrix(self.base_footprint_ground_truth2base_footprint_offset)
        base_footprint_gt2head_mocap = t.compose_matrix(angles=t.euler_from_quaternion(r_base_footprint_gt2head_mocap), translate=t_base_footprint_gt2head_mocap)

        self.head2head_mocap_offset = t.concatenate_matrices(head2base_footprint, base_footprint2base_footprintgt, base_footprint_gt2head_mocap)

        rospy.loginfo_throttle(1,"Head -> Head mocap offset : rotation : {}, translation : {}\n\n\n ".format(*(t.decompose_matrix(self.head2head_mocap_offset)[2:4])))

        self.calibrate_third_loop()

    def calibrate_third_loop(self):
        pass

    def rodriguez_to_quaternion(self, r):
        """
        From http://ros-users.122217.n3.nabble.com/rotation-problem-on-published-tf-links-related-to-a-re-projected-opencv-checkerboard-in-rviz-td2058855.html
        :param r:
        :return:
        """
        mat, _ = cv2.Rodrigues(r)
        w = mat[0][0] + mat[1][1] + mat[2][2] + 1

        w = math.sqrt(w)
        quat = [(mat[2][1] - mat[1][2]) / (w * 2.0),
                (mat[0][2] - mat[2][0]) / (w * 2.0),
                (mat[1][0] - mat[0][1]) / (w * 2.0),
                w / 2.0]
        return quat

    def drawAxis(self, image, corners, image_points):
        corner = tuple(corners[0].ravel())

        image = cv2.line(image, corner, tuple(image_points[0].ravel()), (0, 0, 255), 3)
        image = cv2.line(image, corner, tuple(image_points[1].ravel()), (0, 255, 0), 3)
        image = cv2.line(image, corner, tuple(image_points[2].ravel()), (255, 0, 0), 3)

        return image

    def drawCube(img, corners, imgpts):
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # draw ground floor in green
        img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)

        # draw pillars in blue color
        for i, j in zip(range(4), range(4, 8)):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)

        # draw top layer in red color
        img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)

        return img


# just for convenience
def transformation_matrix(tr, q):
    translation_mat = t.translation_matrix(tr)
    rotation_mat = t.quaternion_matrix(q)
    return np.dot(translation_mat, rotation_mat)

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    mhc = MocapHeadCalib()
    rospy.Timer(rospy.Duration(1), mhc.run)
    rospy.spin()