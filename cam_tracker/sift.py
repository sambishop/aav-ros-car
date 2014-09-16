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
    center = ((max_x + min_x) / 2, (max_y + min_y) / 2)
    print center[0], center[1]
    return center

def find_homography(matches, kp1, kp2):
    query_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    train_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    return cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

sift = cv2.SIFT()
bf = cv2.BFMatcher()

training_image = cv2.imread('car.png', 0)
training_image_keypoints, training_image_descriptors = sift.detectAndCompute(training_image, None)
print '# kps:', len(training_image_keypoints)
training_center = find_center(training_image, training_image_keypoints)

query_image = cv2.imread('car-snapshot.png', 0)
query_image_keypoints, query_image_descriptors = sift.detectAndCompute(query_image, None)
print '# kps:', len(query_image_keypoints)

matches = bf.knnMatch(training_image_descriptors, query_image_descriptors, k=2)
print len(matches)

good_matches = []
for m, n in matches:
    if m.distance < .75 * n.distance:
        good_matches.append(m)
print len(good_matches)

M, mask = find_homography(good_matches, training_image_keypoints, query_image_keypoints)
pts = np.float32([[training_center[0], training_center[1]]]).reshape(-1, 1, 2)
query_center = cv2.perspectiveTransform(pts, M)[0][0]
print query_center
cv2.circle(query_image, (query_center[0], query_center[1]), 4, 255, -1)

#plt.imshow(query_image)
#plt.show()
