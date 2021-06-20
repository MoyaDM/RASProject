import cv2

cap = cv2.VideoCapture(0)

x = 1

while(1):
    ret, frame = cap.read()

    cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
    cv2.imshow('Video', frame)

    if ((cv2.waitKey(33) == -1) and (x == 1)):
        print ("Ninguna")
        x = 0

    if (cv2.waitKey(33) == ord('w')):
        print ("Upkey")
        x = 1

    if (cv2.waitKey(33) == ord('a')):
        print ("Leftkey")
        x = 1
    
    if (cv2.waitKey(33) == ord('s')):
        print ("Dowkey")
        x = 1

    if (cv2.waitKey(33) == ord('d')):
        print ("Rightkey")
        x = 1

    if cv2.waitKey(33) == 32:
        break

