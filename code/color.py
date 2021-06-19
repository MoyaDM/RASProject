import cv2

frame = cv2.imread("../img/opencv.png")
frame = cv2.resize(frame, (300,300))
frame_flip = cv2.flip(frame,0)
frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

(B, G, R) = cv2.split(frame)

r_channel=frame.copy()
r_channel[:,:,0] = 0
r_channel[:,:,1] = 0

b_channel=frame.copy()
b_channel[:,:,1] = 0
b_channel[:,:,2] = 0

g_channel=frame.copy()
g_channel[:,:,0] = 0
g_channel[:,:,2] = 0

cv2.imshow("Red", r_channel)
cv2.imshow("Blue", b_channel)
cv2.imshow("Green", g_channel)
cv2.imshow("B", B)
cv2.imshow("G", G)
cv2.imshow("R", R)
cv2.imshow("Frame", frame)
cv2.imshow("Flip", frame_flip)
cv2.imshow("Gray", frame_gray)
cv2.waitKey(0)
cv2.destroyAllWindows()
