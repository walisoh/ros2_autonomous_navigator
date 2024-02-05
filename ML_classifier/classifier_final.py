import rclpy
import os.path
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
import sys

import numpy as np
import cv2
import csv
from cv_bridge import CvBridge


class ClassifyObject(Node):
    def __init__(self):
        # Create find_object Node
        super().__init__('find_object')

        # Declare Parameters of Node
        self.declare_parameter('window_name', 'Base Image')
        self.declare_parameter('show_image_bool', True)

        # Get Parameters of Node
        self.windowName = self.get_parameter('window_name').value
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # Declare node is subscribing to 'camera/image/compressed' topic
        self._camera_subscriber = self.create_subscription(CompressedImage, 'camera/image/compressed',
                                                           self._image_classifier, 1)
        self._camera_subscriber

        # Declare node is publishing to 'coordinate' topic
        self._knn_publisher = self.create_publisher(Point, '/coord', 1)
        self._knn_publisher

        # self.knn = cv2.ml.KNearest_create()

        # self.imagefolder = './trained_model.yml/'
        # self.model = knn.load(os.path(self.imagefolder)) #check if its more than path

        self.model = cv2.ml.KNearest_create()
        fs = cv2.FileStorage('src/labfinal/labfinal/trained_model.yml', cv2.FILE_STORAGE_READ)
        knn_yml = fs.getNode('opencv_ml_knn')

        knn_format = knn_yml.getNode('format').real()
        is_classifier = knn_yml.getNode('is_classifier').real()
        default_k = knn_yml.getNode('default_k').real()
        samples = knn_yml.getNode('samples').mat()
        responses = knn_yml.getNode('responses').mat()
        fs.release
        self.model.train(samples,cv2.ml.ROW_SAMPLE, responses)


    def _image_classifier(self,CompressedImage):

        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")

        pt = Point()
        pt.x = 0.0

        k = 5

        hsv_test = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)
        mask1_test = cv2.inRange(hsv_test, (0, 90, 0), (180, 255, 255))  # could do 90-255 saturation
        mask_test = mask1_test
        contours_test, hierarchy = cv2.findContours(mask_test.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours_test) != 0:
            # draw in blue the contours that were founded
            cv2.drawContours(self._imgBGR, contours_test, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = sorted(contours_test, key=cv2.contourArea, reverse=True)[:1]
            x, y, w, h = cv2.boundingRect(c[0])

            # draw the biggest contour (c) in green
            cv2.rectangle(self._imgBGR, (x, y), (x + w, y + h), (0, 255, 0), 2)
            rect_area = w * h
            print('rect area: ', rect_area)
            crop_test = self._imgBGR[y:y + h, x:x + w]

        else:
            print('peepee')
            rect_area = 1999


        mask_img = np.array(cv2.resize(mask_test, (40, 40)))
        test_img = np.array(cv2.resize(crop_test, (40, 40)))

        # if (__debug__):
        #     cv2.imshow(Title_images, test_img)
        #     cv2.imshow(Title_resized, mask_img)
        #     key = cv2.waitKey()
        #     if key == 27:  # Esc key to stop
        #         break

        test_img = test_img.flatten().reshape(1, -1)
        test_img = test_img.astype(np.float32)

        if rect_area < 2000:
            ret = 0
        else:
            ret, results, neighbours, dist = self.model.findNearest(test_img, k)

        pt.x =  float(ret)
        self._knn_publisher.publish(pt)

        print(" classified as " + str(ret))
        # print("\tneighbours: " + str(neighbours))
        # print("\tdistances: " + str(dist))

def main():
    rclpy.init()
    obj = ClassifyObject()
    while rclpy.ok():
        rclpy.spin_once(obj)  # Trigger callback processing.
    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()