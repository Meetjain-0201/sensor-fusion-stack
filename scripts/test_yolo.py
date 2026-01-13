#!/usr/bin/env python3

from ultralytics import YOLO
import cv2

# Load model
model = YOLO('yolov8n.pt')

# Test on sample image
print("Testing YOLO on sample image...")

# Download a test image
import urllib.request
url = 'https://ultralytics.com/images/bus.jpg'
urllib.request.urlretrieve(url, '/tmp/test_image.jpg')

# Run detection
img = cv2.imread('/tmp/test_image.jpg')
results = model(img, conf=0.3)

# Display results
for result in results:
    boxes = result.boxes
    print(f"Detected {len(boxes)} objects:")
    for box in boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        class_name = model.names[cls]
        print(f"  - {class_name}: {conf:.2f}")

# Save annotated image
annotated = results[0].plot()
cv2.imwrite('/tmp/test_annotated.jpg', annotated)
print("Annotated image saved to /tmp/test_annotated.jpg")
print("Open it with: eog /tmp/test_annotated.jpg")
