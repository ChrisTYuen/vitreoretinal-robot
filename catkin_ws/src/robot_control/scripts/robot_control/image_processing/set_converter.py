#!/usr/bin/python3
import math
import numpy as np
import cv2

# ROS
# import rospy

mouse_output_list_1 = []
mouse_output_list_2 = []
num_of_point_1 = 0
num_of_point_2 = 0
pointing_is_over = False


def capture_mouse_position(event, x, y, flags, param):
    global mouse_output_list_1
    global mouse_output_list_2
    global num_of_point_1
    global num_of_point_2
    global pointing_is_over

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_output_list_1.append((x, y))
        # print((x, y))
        num_of_point_1 = num_of_point_1 + 1
        print("left" + str(len(mouse_output_list_1)))

    if event == cv2.EVENT_RBUTTONDOWN:
        mouse_output_list_2.append((x, y))
        # print((x, y))
        num_of_point_2 = num_of_point_2 + 1
        print("right" + str(len(mouse_output_list_2)))


try:
    # rospy.init_node("set_converter", disable_signals=True)

    microscopic_image = cv2.imread("/home/yuki/set_converter.png")
    microscopic_image = microscopic_image[600:2000, 500:3000]

    cv2.namedWindow('microscopic_image')
    cv2.setMouseCallback('microscopic_image', capture_mouse_position)
    cv2.imshow('microscopic_image', microscopic_image)

    k = cv2.waitKey(0)

    print("Converter set is done!!")

    points_1 = np.array(mouse_output_list_1)
    points_2 = np.array(mouse_output_list_2)
    x_1 = points_1[:, 0]
    y_1 = points_1[:, 1]
    x_2 = points_2[:, 0]
    y_2 = points_2[:, 1]

    a_1, b_1 = np.polyfit(x_1, y_1, 1)
    a_2, b_2 = np.polyfit(x_2, y_2, 1)

    print("a_1:" + str(a_1) + ", a_2:" + str(a_2))

    print("b_1:" + str(b_1) + ", b_2:" + str(b_2))

    thickness_px = np.linalg.norm(1/np.sqrt(a_1**2+1)*(a_1*(x_2[0]-x_1[0])-(y_2[0]-y_1[0])))

    print("thickness_px: " + str(thickness_px) + " px")

    converter = thickness_px/0.51

    print("converter is " + str(converter) + " px/mm")

    # rospy.set_param("px_mm_converter", str(converter))

    if k == ord('q'):
        cv2.destroyAllWindows()

except Exception as exp:
    print("[" + "]:: {}.".format(exp))
    # print("[" + rospy.get_name() + "]:: {}.".format(exp))

except KeyboardInterrupt:
    cv2.destroyAllWindows()
    print("Keyboard Interrupt!!")

