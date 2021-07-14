import cv2
captura = cv2.VideoCapture(0)
salida = cv2.VideoWriter('videoSalida.mp4',cv2.VideoWriter_fourcc(*'XVID'),20.0,(640,480))
while (captura.isOpened()):
  ret, imagen = captura.read()
  if ret == True:
    cv2.imshow('video', imagen)
    salida.write(imagen)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  else: break
captura.release()
salida.release()
cv2.destroyAllWindows()
