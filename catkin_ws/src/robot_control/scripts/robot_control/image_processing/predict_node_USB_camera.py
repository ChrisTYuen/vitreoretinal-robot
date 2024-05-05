#!/usr/bin/python3
import os
import torch
import numpy as np
import cv2
from tools import predict_functions as functions
import segmentation_models_pytorch as smp

import rospy
import time
from rosilo_datalogger.msg import AddValueMsg

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('predict')
    predict_node = PredictNode()
    rospy.spin()

class PredictNode:
    def __init__(self):
        self.publisher_distance_ = rospy.Publisher("predict/distance",AddValueMsg,queue_size=1)

        self.best_model = torch.load('/home/yuki/Documents/eye_surgery/image_processing/data_tip/result/confidencemap_cropped_512_512_resnet18/best_model.pth')
        self.ENCODER = 'resnet18'
        self.ENCODER_WEIGHTS = 'imagenet'
        self.DEVICE = 'cuda'
        self.preprocessing_fn = smp.encoders.get_preprocessing_fn(self.ENCODER, self.ENCODER_WEIGHTS)
        self.preprocessing = functions.get_preprocessing(self.preprocessing_fn)

        self.bridge = CvBridge()

        self.shaft_distance_last = 100
        self.tip_distance_last = 100
        self.x1_last = 0
        self.x2_last = 0
        self.x3_last = 0
        self.y1_last = 0
        self.y2_last = 0
        self.y3_last = 0
        self.iteration = 0

        self.kernel = np.ones((5, 5), np.uint8)

        self.counter_window = np.zeros([5])

        self.subscriber_microscopic_image_ = rospy.Subscriber("microscope/image", Image, self._get_image)
#        self.subscriber_microscopic_image_ = rospy.Subscriber("sas/decklink/0/get/video", Image, self._get_image)

        print("[predict_node]::Ready!")

    def _get_image(self,msg):
        start = time.time()
        self.iteration = self.iteration + 1
        orig = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = self.preprocessing(image=orig)['image']
        x_tensor = torch.from_numpy(frame).to(self.DEVICE).unsqueeze(0)
        pr_mask = self.best_model.predict(x_tensor)
        pr_mask = (pr_mask.squeeze().cpu().numpy().round())
        pr_mask = np.transpose(pr_mask, (1, 2, 0))

#        pr_mask[:, :, 0] = cv2.morphologyEx(pr_mask[:, :, 0], cv2.MORPH_OPEN, self.kernel)
#        pr_mask[:, :, 1] = cv2.morphologyEx(pr_mask[:, :, 1], cv2.MORPH_OPEN, self.kernel)

        x1, y1 = functions.get_instrument_tip(pr_mask[:, :, 0],self.x1_last,self.y1_last,self.iteration)
        x2, y2 = functions.get_shadow_tip(pr_mask[:, :, 1],self.x2_last,self.y2_last,self.iteration)
        x3, y3 = functions.get_instrument_other_end(pr_mask[:, :, 0],self.x3_last,self.y3_last,self.iteration)

        self.x1_last = x1
        self.x2_last = x2
        self.x3_last = x3
        self.y1_last = y1
        self.y2_last = y2
        self.y3_last = y3

        pr_mask_output = np.ones(pr_mask.shape) * 200
        for jj in range(pr_mask.shape[2]):
            pr_mask_output[:, :, jj] = pr_mask_output[:, :, jj] - 200 * pr_mask[:, :, 0] - 100 * pr_mask[:, :, 1]

        tip_distance = np.sqrt((x1-x2)**2+(y1-y2)**2)
        cv2.putText(pr_mask_output, 'tip_distance:' + str(int(tip_distance)) + ' pixel', (10, 40),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        self.counter_window = np.delete(self.counter_window,0,None)
        if tip_distance > 1:
            self.counter_window = np.append(self.counter_window,0)
        else:
            self.counter_window = np.append(self.counter_window,1)

        counter_sum = np.sum(self.counter_window)

        shaft_distance = np.abs((x2-x1)*(y1-y3)-(y2-y1)*(x1-x3))/np.sqrt((y1-y3)**2+(x1-x3)**2)
        cv2.putText(pr_mask_output, 'shaft_distance:' + str(int(shaft_distance)) + ' pixel', (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.circle(pr_mask_output, (x1, y1), 2, (0, 0, 255), -1)
        cv2.circle(pr_mask_output, (x2, y2), 2, (255, 0, 0), -1)

        pr_mask_output = pr_mask_output.astype('uint8')
        output = cv2.hconcat([orig, pr_mask_output])

        cv2.imshow('img_test', output)
        cv2.waitKey(1)

        msg = AddValueMsg(name="distance",value=[tip_distance,shaft_distance,counter_sum])
        self.publisher_distance_.publish(msg)

#        print(tip_distance)

        self.tip_distance_last = tip_distance
        self.shaft_distance_last = shaft_distance
#        print(time.time()-start)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
