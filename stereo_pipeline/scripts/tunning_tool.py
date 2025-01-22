import cv2
def trackbar_callback(x):
    pass

cv2.namedWindow("Tuning")
cv2.createTrackbar("Threshold", "Tuning", 50, 255, trackbar_callback)

while True:
    threshold = cv2.getTrackbarPos("Threshold", "Tuning")
    # Aplicação do threshold
    cv2.imshow("Result", processed_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

