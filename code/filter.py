import cv2

frame = cv2.imread("../img/papas.jpg")

kernel_size = (25,25)
blur = cv2.GaussianBlur(frame, kernel_size, 0)


for i in range(20):
    blur = cv2.GaussianBlur(blur, kernel_size, 0)

cv2.imshow("Frame", frame)
cv2.imshow("Blur", blur)
cv2.waitKey(0)
cv2.destroyAllWindows()
