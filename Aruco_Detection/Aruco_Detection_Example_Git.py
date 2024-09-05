import cv2
import cv2.aruco as aruco

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

cap = cv2.VideoCapture(0)
print(cap.isOpened())
while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv2.cvtColor (frame,cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(marker_dict, parameters)
    marker_corners, marker_IDs, reject = detector.detectMarkers(
    gray_frame)
    print(marker_IDs)
    cv2. imshow ("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("g"):
        break
cap.release()
cv2.destroyAllWindows()