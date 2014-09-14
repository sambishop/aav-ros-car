#!/usr/bin/python

import numpy as np
import cv2
from matplotlib import pyplot as plt

def find_center(image, keypoints):
    min_y, min_x = image.shape
    max_y, max_x = (0, 0)
    for kp in keypoints:
        min_x, max_x = min(min_x, kp.pt[0]), max(max_x, kp.pt[0])
        min_y, max_y = min(min_y, kp.pt[1]), max(max_y, kp.pt[1])
    center = (int((max_x + min_x) / 2), int((max_y + min_y) / 2))
    return center

def find_homography(matches, kp1, kp2):
    query_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    train_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    return cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

training_image = cv2.imread('car.png', 0)
query_image = cv2.imread('car-snapshot.png', 0)

sift = cv2.SIFT()
training_image_keypoints, training_image_descriptors = sift.detectAndCompute(training_image, None)
query_image_keypoints, query_image_descriptors = sift.detectAndCompute(query_image, None)

training_center = find_center(training_image, training_image_keypoints)

bf = cv2.BFMatcher()
matches = bf.knnMatch(training_image_descriptors, query_image_descriptors, k=2)

good_matches = []
for m, n in matches:
    if m.distance < .75 * n.distance:
        good_matches.append(m)

M, mask = find_homography(good_matches, training_image_keypoints, query_image_keypoints)
query_center = cv2.perspectiveTransform(np.float32([[training_center[0], training_center[1]]]).reshape(-1, 1, 2), M)
print query_center

#h, w = training_image.shape
#pts1 = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]])
#print pts1.shape
#pts2 = pts1.reshape(-1, 1, 2)
#print pts2.shape
#dst = cv2.perspectiveTransform(pts2, M)
#cv2.polylines(query_image, [np.int32(dst)], True, 255, 3)


plt.imshow(query_image)
plt.show()

#cv2.imwrite('sift_keypoints.jpg', img3)

