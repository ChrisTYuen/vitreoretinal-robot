#!/usr/bin/python3
import cv2
import numpy as np
from time import sleep
from tools import img_parameters

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sas_datalogger.msg import AddValueMsg
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
    """
    This class is used to get the 5 positioning points from the image. 
    """
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

        cut_out_images = []

        for i in range(5):
            image_cut_out, top, left = self.cut_out_image(overview_image, i + 1)
            cut_out_images.append(image_cut_out)
            hsvMaskOpening = self.hsvExtraction(image_cut_out)
            point_contours = self.get_contours(hsvMaskOpening)
            if not self.lock_or_not:
                self.centers[i, :], self.radius[i, 0] = self.get_center_and_radius_of_specified_point(point_contours,
                                                                                                    image_cut_out,
                                                                                                    top, left, i + 1)

        center_and_radius = np.vstack([self.centers.reshape([2*self.num_of_points, 1]),
                                       self.radius.reshape([self.num_of_points, 1])])
        msg = AddValueMsg(name="points_in_order", value=center_and_radius*self.ratio)

        if self.count == self.publish_count:
            if not self.lock_or_not:
                self.publisher_positioning_points_.publish(msg)

            self.count = 0

            if self.image_debug:
                for i in range(5):
                    cv2.imshow(f'image_cut_out_{i}', cut_out_images[i])
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
        num_of_points = len(centers) 
        centers_array = np.array(centers)
        center_index_array = np.full(num_of_points, -1, dtype=int)

        # Assigning indices based on the minimum and maximum values
        center_index_array[0] = np.argmin(centers_array[:, 1])
        center_index_array[1] = np.argmin(centers_array[:, 0])
        center_index_array[3] = np.argmax(centers_array[:, 0])
        center_index_array[4] = np.argmax(centers_array[:, 1])
        center_index_array[2] = np.argmin(center_index_array)

        centers_in_order = np.zeros([num_of_points, 2])
        radius_in_order = np.zeros([num_of_points, 1])

        # Loop through the center_index_array to assign values to centers_in_order and radius_in_order
        for i in range(num_of_points):
            index = int(center_index_array[i])
            centers_in_order[i] = self.set_center(centers[index], i)
            radius_in_order[i] = radius[index]

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
        # Default values for the entire image
        coordinates = [
            (0, int(image_shape[0]*3/8), int(image_shape[1]/5), int(image_shape[1]*4/5)),  # Point 1
            (0, image_shape[0], int(image_shape[1]/5), int(image_shape[1]*2/5)),  # Point 2
            (int(image_shape[0]/3), int(image_shape[0]*2/3), int(image_shape[1]/3), int(image_shape[1]*2/3)),  # Point 3
            (0, image_shape[0], int(image_shape[1]*5/8), image_shape[1]),  # Point 4
            (int(image_shape[0]*5/8), image_shape[0], int(image_shape[1]/5), int(image_shape[1]*3/5))  # Point 5
        ]

        # Adjust the index to match the point numbering starting from 1
        top, bottom, left, right = coordinates[point - 1]

        image_cut_out = image[top:bottom, left:right]

        return image_cut_out, top, left


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

