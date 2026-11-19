from deepface import DeepFace
import numpy as np
import cv2

cap  = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    results = DeepFace.analyze(frame,
                     actions = ['age', 'gender', 'race', 'emotion'],
                     enforce_detection=False)
    for face in results:
        print(f"Dominant Emotion: {face['dominant_emotion']}")
        print(f"Age: {face['age']}")
        print(f"Gender: {face['dominant_gender']}")
        print(f"Race: {face['dominant_race']}")
    cv2.imshow('My Video', frame)
    if cv2.waitKey(10) & 0xFF == 27:
        break
cap.release()
cv2.destroyAllWindows()
