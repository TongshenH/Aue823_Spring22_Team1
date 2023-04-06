import cv2
import numpy as np

img = cv2.imread("autorace.png")

def mask_img(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask,vertices, color=(255, 255, 255))
    masked_image=cv2.bitwise_and(img,mask)
    return masked_image

height = img.shape[0]
width = img.shape[1]
# roi_vertices = np.int32(np.array([(0,height),(5*width/10,6*height/10),(width,height)]))
roi_vertices = np.array([[0,height],[5*width//10,6*height//10], [width,height]])

masked_img = roi(img, [roi_vertices])

cv2.imshow("Masked", masked_img)
cv2.waitKey(0)