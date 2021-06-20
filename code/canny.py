import numpy as np
import matplotlib.pyplot as plt
import cv2
import math

img = cv2.imread("../img/Emma.jpg")
img = cv2.resize(img,(600,800))
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

kernel_size = (25,25)
gauss = cv2.GaussianBlur(img_gray, kernel_size, 0)

umbral_minimo = 85
umbral_maximo = 275

canny = cv2.Canny(img_gray, umbral_minimo, umbral_maximo)

cv2.imshow("Original", img)
cv2.imshow("Grises", img_gray)
cv2.imshow("Canny", canny)

cv2.waitKey(0)
cv2.destroyAllWindows() 