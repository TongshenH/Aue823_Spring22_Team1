import torch
import cv2

def draw_lines(xmin, xmax, ymin, ymax, cv_img):
    # top
    cv2.line(cv_img, (xmin, ymax), (xmax, ymax), (0,255,0) ,2)
    # bottom
    cv2.line(cv_img, (xmin, ymin), (xmax, ymin), (0,255,0), 2)
    # left
    cv2.line(cv_img, (xmin, ymin), (xmin, ymax), (0,255,0), 2)
    # right
    cv2.line(cv_img, (xmax, ymin), (xmax, ymax), (0,255,0), 2)
    
# Model
model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom

# Images
img = "stop_sign.jpg"  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img)

# Results
results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
data = results.pandas().xyxy[0]

is_stop = True if 11 in data['class'].values else False
row = -1
if is_stop:
    for i, row in enumerate(data.loc[:,'class']):
        if row == 11:
            is_stop = True
            row = data.iloc[i]
            break

xmin = int(row.loc['xmin'])
xmax = int(row.loc['xmax'])
ymin = int(row.loc['ymin'])
ymax = int(row.loc['ymax'])

cv_img = cv2.imread("stop_sign.jpg")

draw_lines(xmin, xmax, ymin, ymax, cv_img)

cv2.imshow("Custom Lines", cv_img)
cv2.waitKey(0)