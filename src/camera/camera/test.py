import cv2
from ultralytics import YOLO

# Load the YOLO instance segmentation model
model = YOLO("best.pt")  # Replace with your model path if custom

# Open the webcam (0 = default camera)
cap = cv2.VideoCapture(0)

# Check if webcam opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO inference on the frame
    results = model.predict(source=frame, show=False, stream=True)

    for result in results:
        # Draw masks and boxes on frame
        annotated_frame = result.plot()

    # Show the output frame
    cv2.imshow("YOLOv8 Instance Segmentation", annotated_frame)

    # Exit loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
