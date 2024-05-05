#!/usr/bin/python3
import cv2
import numpy as np
from time import sleep
from tools import img_parameters

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rosilo_datalogger.msg import AddValueMsg
from std_msgs.msg import Bool
from std_msgs.msg import String

import rospy


def main():
    # Init node
    rospy.init_node('get_positioning_points')
    get_positioning_points = GetPositioningPoints()
    rospy.spin()
    cv2.destroyAllWindows()


class GetPositioningPoints:
    def __init__(self):
        self.parameters = img_parameters.ImageParameters
        self.image_debug = self.parameters.image_debug
        self.min_point_area = self.parameters.min_point_area

        self.hsvLower1 = self.parameters.hsvLower1
        self.hsvUpper1 = self.parameters.hsvUpper1
        self.hsvLower2 = self.parameters.hsvLower2
        self.hsvUpper2 = self.parameters.hsvUpper2

        self.erode_iteration = self.parameters.erode_iteration
        self.dilate_iteration = self.parameters.dilate_iteration

        self.ratio = self.parameters.original_h / self.parameters.output_size

        self.num_of_points = self.parameters.num_of_points
        self.centers_one_step_before = [None]*self.num_of_points

        # Launch ROS subscribers
        self.subscriber_output_image_ = rospy.Subscriber("img_show/overview", Image, self._img_show_callback)
        self.subscriber_tip_positions_ = rospy.Subscriber("predict/tip_positions", AddValueMsg, self._tip_positions_callback)
        # self.subscriber_target_lock_ = rospy.Subscriber("predict/lock_or_not", Bool, self._target_lock_callback)
        self.publisher_positioning_points_ = rospy.Publisher("predict/positioning_points", AddValueMsg, queue_size=10)
        self.publisher_current_step = rospy.Publisher("predict/current_step", String, queue_size=10)
        self.publisher_planar_error = rospy.Publisher("predict/planar_error", AddValueMsg, queue_size=10)

        self.bridge = CvBridge()
        self.bridge_to_cv2 = self.bridge.imgmsg_to_cv2

        self.count = 0
        self.publish_count = self.parameters.update_pace

        self.next_point = 0
        rospy.set_param("/next_point", [0])

        rospy.set_param("/target_lock", False)

        rospy.set_param("/step", "Waiting")

        self.centers = np.zeros([self.num_of_points, 2])
        self.radius = np.zeros([self.num_of_points, 1])

        print("[" + rospy.get_name() + "]:: Ready!")

    # def _target_lock_callback(self, msg):
    #     self.lock_or_not = msg

    def _img_show_callback(self, msg):
        overview_image = self.bridge_to_cv2(msg, "bgr8")

        self.lock_or_not = rospy.get_param("/target_lock")

        image_cut_out_1, top, left = self.cut_out_image(overview_image, 1)
        hsvMaskOpening_1 = self.hsvExtraction(image_cut_out_1)
        point_contours = self.get_contours(hsvMaskOpening_1)
        if not self.lock_or_not:
            self.centers[0, :], self.radius[0, 0] = self.get_center_and_radius_of_specified_point(point_contours,
                                                                                                  image_cut_out_1,
                                                                                                  top, left, 1)

        image_cut_out_2, top, left = self.cut_out_image(overview_image, 2)
        hsvMaskOpening_2 = self.hsvExtraction(image_cut_out_2)
        point_contours = self.get_contours(hsvMaskOpening_2)
        if not self.lock_or_not:
            self.centers[1, :], self.radius[1, 0] = self.get_center_and_radius_of_specified_point(point_contours,
                                                                                                  image_cut_out_2,
                                                                                                  top, left, 2)

        image_cut_out_3, top, left = self.cut_out_image(overview_image, 3)
        hsvMaskOpening_3 = self.hsvExtraction(image_cut_out_3)
        point_contours = self.get_contours(hsvMaskOpening_3)
        if not self.lock_or_not:
            self.centers[2, :], self.radius[2, 0] = self.get_center_and_radius_of_specified_point(point_contours,
                                                                                                  image_cut_out_3,
                                                                                                  top, left, 3)

        image_cut_out_4, top, left = self.cut_out_image(overview_image, 4)
        hsvMaskOpening_4 = self.hsvExtraction(image_cut_out_4)
        point_contours = self.get_contours(hsvMaskOpening_4)
        if not self.lock_or_not:
            self.centers[3, :], self.radius[3, 0] = self.get_center_and_radius_of_specified_point(point_contours,
                                                                                                  image_cut_out_4,
                                                                                                  top, left, 4)

        image_cut_out_5, top, left = self.cut_out_image(overview_image, 5)
        hsvMaskOpening_5 = self.hsvExtraction(image_cut_out_5)
        point_contours = self.get_contours(hsvMaskOpening_5)
        if not self.lock_or_not:
            self.centers[4, :], self.radius[4, 0] = self.get_center_and_radius_of_specified_point(point_contours,
                                                                                                  image_cut_out_5,
                                                                                                  top, left, 5)

        center_and_radius = np.vstack([self.centers.reshape([2*self.num_of_points, 1]),
                                       self.radius.reshape([self.num_of_points, 1])])
        msg = AddValueMsg(name="points_in_order", value=center_and_radius*self.ratio)

        if self.count == self.publish_count:
            if not self.lock_or_not:
                self.publisher_positioning_points_.publish(msg)

            self.count = 0

            if self.image_debug:
                cv2.imshow('image_cut_out_1', image_cut_out_1)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('image_cut_out_2', image_cut_out_2)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('image_cut_out_3', image_cut_out_3)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('image_cut_out_4', image_cut_out_4)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('image_cut_out_5', image_cut_out_5)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('HSV_mask_opening_1', hsvMaskOpening_1)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('HSV_mask_opening_2', hsvMaskOpening_2)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('HSV_mask_opening_3', hsvMaskOpening_3)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('HSV_mask_opening_4', hsvMaskOpening_4)
                cv2.waitKey(1)

            if self.image_debug:
                cv2.imshow('HSV_mask_opening_5', hsvMaskOpening_5)
                cv2.waitKey(1)
        else:
            self.count = self.count + 1

        current_step = rospy.get_param("/step")
        self.publisher_current_step.publish(current_step)

    def _tip_positions_callback(self, msg):
        positions = msg.value
        current_tip_position = np.array([positions[6], positions[7]])
        self.next_point = rospy.get_param("next_point")
        next_point = self.next_point[0]
        msg = AddValueMsg(value=np.array([np.linalg.norm(self.centers[next_point, :]*self.ratio-current_tip_position)]))
        self.publisher_planar_error.publish(msg)


    def bgrExtraction(self, image, bgrLower, bgrUpper):
        img_mask = cv2.inRange(image, bgrLower, bgrUpper)  # BGRからマスクを作成
        result = cv2.bitwise_and(image, image, mask=img_mask)  # 元画像とマスクを合成
        return result


    def hsvExtraction(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Convert to HSV
        hsv_mask1 = cv2.inRange(hsv, self.hsvLower1, self.hsvUpper1)  # make mask1
        hsv_mask2 = cv2.inRange(hsv, self.hsvLower2, self.hsvUpper2)  # make mask2
        hsv_mask = hsv_mask1 + hsv_mask2
        hsv_mask_erosion = cv2.erode(hsv_mask, np.ones((2, 2), np.uint8), iterations=self.erode_iteration)
        hsv_mask_opening = cv2.dilate(hsv_mask_erosion, np.ones((2, 2), np.uint8), iterations=self.dilate_iteration)
        # result = cv2.bitwise_and(image, image, mask=hsv_mask)  # 元画像とマスクを合成
        return hsv_mask_opening


    def get_contours(self, mask_image):
        contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        point_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_point_area]
        # if self.image_debug:
            # print('number of points: %d' % len(point_contours))
        return point_contours


    def get_max_area_contour(self, mask_image):
        contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_value = 0
        for cnt in contours:
            if cv2.contourArea(cnt) > max_value:
                max_contour = cnt
                max_value = cv2.contourArea(cnt)
        return max_contour


    def draw_circles(self, point_contours, image):
        contours_poly = [None] * len(point_contours)
        centers = [None] * len(point_contours)
        radius = [None] * len(point_contours)

        for i, c in enumerate(point_contours):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])

        if self.image_debug:
            for i in range(len(point_contours)):
                cv2.circle(image, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), (255, 0, 0), 2)
                cv2.circle(image, (int(centers[i][0]), int(centers[i][1])), 3, (0, 255, 0), -1)
        return centers, radius


    def get_center_and_radius_of_specified_point(self, point_contours, image, top, left, point):
        image_shape = image.shape
        bottom = image_shape[0]
        right = image_shape[1]

        center = (right/2, bottom/2)
        radius = 10

        contours_poly = [None] * len(point_contours)
        centers = [None] * len(point_contours)
        radii = [None] * len(point_contours)

        for i, c in enumerate(point_contours):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            centers[i], radii[i] = cv2.minEnclosingCircle(contours_poly[i])

        if point == 1:
            dis = bottom
            for j in range(len(point_contours)):
                if centers[j][1] < dis:
                    center = centers[j]
                    radius = radii[j]
                    dis = centers[j][1]
        elif point == 2:
            dis = right
            for j in range(len(point_contours)):
                if centers[j][0] < dis:
                    center = centers[j]
                    radius = radii[j]
                    dis = centers[j][0]
        elif point == 3:
            dis = (right/2)**2 + (bottom/2)**2
            for j in range(len(point_contours)):
                if (centers[j][0] - right/2)**2 + (centers[j][1] - bottom/2)**2 < dis:
                    center = centers[j]
                    radius = radii[j]
                    dis = (centers[j][0] - right/2)**2 + (centers[j][1] - bottom/2)**2
        elif point == 4:
            dis = 0
            for j in range(len(point_contours)):
                if centers[j][0] > dis:
                    center = centers[j]
                    radius = radii[j]
                    dis = centers[j][0]
        elif point == 5:
            dis = 0
            for j in range(len(point_contours)):
                if centers[j][1] > dis:
                    center = centers[j]
                    radius = radii[j]
                    dis = centers[j][1]

        if self.image_debug:
            cv2.circle(image, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)
            cv2.circle(image, (int(center[0]), int(center[1])), 3, (0, 255, 0), -1)

        center = (center[0] + left, center[1] + top)

        return center, radius

    def get_points_in_order(self, centers, radius):
        centers_array = np.zeros([len(centers), 2])
        center_index_array = np.ones([self.num_of_points])*(-1)
        centers_in_order = np.zeros([self.num_of_points, 2])
        radius_in_order = np.zeros([self.num_of_points, 1])
        for i in range(len(centers)):
            centers_array[i][0] = centers[i][0]
            centers_array[i][1] = centers[i][1]

        center_index_array[0] = np.argmin(centers_array[:, 1])
        center_index_array[1] = np.argmin(centers_array[:, 0])
        center_index_array[3] = np.argmax(centers_array[:, 0])
        center_index_array[4] = np.argmax(centers_array[:, 1])
        center_index_array[2] = np.argmin(center_index_array)

        centers_in_order[0] = self.set_center(centers[int(center_index_array[0])], 0)
        centers_in_order[1] = self.set_center(centers[int(center_index_array[1])], 1)
        centers_in_order[2] = self.set_center(centers[int(center_index_array[2])], 2)
        centers_in_order[3] = self.set_center(centers[int(center_index_array[3])], 3)
        centers_in_order[4] = self.set_center(centers[int(center_index_array[4])], 4)

        radius_in_order[0] = radius[int(center_index_array[0])]
        radius_in_order[1] = radius[int(center_index_array[1])]
        radius_in_order[2] = radius[int(center_index_array[2])]
        radius_in_order[3] = radius[int(center_index_array[3])]
        radius_in_order[4] = radius[int(center_index_array[4])]

        return centers_in_order*self.ratio, radius_in_order*self.ratio

    def set_center(self, new_center, point_num):
        if self.centers_one_step_before[point_num] == None:
            center = new_center
            self.centers_one_step_before[point_num] = new_center
        else:
            dif = np.linalg.norm(np.array([new_center]) - np.array([self.centers_one_step_before[point_num]]))
            if dif > 5:
                center = self.centers_one_step_before[point_num]
            else:
                center = new_center
                self.centers_one_step_before[point_num] = new_center
        return np.array([center])

    def cut_out_image(self, image, point):
        image_shape = image.shape
        top = 0
        left = 0
        bottom = image_shape[0]
        right = image_shape[1]
        if point == 1:
            top = 0
            bottom = int(image_shape[0]*3 / 8)
            left = int(image_shape[1]/5)
            right = int(image_shape[1]*4/5)
        elif point == 2:
            top = 0
            bottom = int(image_shape[0])
            left = int(image_shape[1] / 5)
            right = int(image_shape[1]*2 / 5)
        elif point == 3:
            top = int(image_shape[0] / 3)
            bottom = int(image_shape[0] * 2 / 3)
            left = int(image_shape[1] / 3)
            right = int(image_shape[1] * 2 / 3)
        elif point == 4:
            top = 0
            bottom = int(image_shape[0])
            left = int(image_shape[1]*5 / 8)
            right = int(image_shape[1]*4/4)
        elif point == 5:
            top = int(image_shape[0]*5 / 8)
            bottom = int(image_shape[0])
            left = int(image_shape[1]/5)
            right = int(image_shape[1]*3/5)

        image_cut_out = image[top: bottom, left: right]

        return image_cut_out, top, left


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

