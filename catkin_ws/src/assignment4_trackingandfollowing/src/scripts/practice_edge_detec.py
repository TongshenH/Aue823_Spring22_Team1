import cv2

image = cv2.imread("blob_detec.png")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 20, 10, apertureSize=3)

cv2.imshow("Edges", edges)

cv2.waitKey(0)