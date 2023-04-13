# import torch
import numpy as np
import cv2
import torch

model = torch.hub.load('/home/gbbyrd/.local/lib/python3.8/site-packages/yolov5', 'custom', path='/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/src/auefinals/src/scripts/yolov5s.pt', source='local')
img = cv2.imread('/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/src/auefinals/src/scripts/soccer.jpg')
# img = cv2.cvtColor(img, cv2.cvtColor())
cv2.imshow('yes', img)
output = model(img)
output.show()
print('did it work?')

def performOperations(arr, operations):
    # Write your code here
    for operation in operations:
        l = operation[0]
        r = operation[1]
        while l < r:
            arr[l], arr[r] = arr[r], arr[l]
            l += 1
            r -= 1
            
arr = [9, 8, 7, 6, 5, 4, 3, 2, 1]
operations = [[0,9], [4,5], [3,6],[2,7],[1,8],[0,9]]



