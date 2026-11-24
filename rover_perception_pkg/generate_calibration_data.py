import os
import cv2
import numpy as np
from ultralytics import YOLO

# ==========================================
# 1. SETUP YOUR PATHS HERE
# ==========================================
MODEL_PATH = "weights/sim_to_real_alpha_best.pt"                   # Path to your trained YOLO weights
VAL_IMAGES_DIR = "dataset/valid/images"   # Folder with your validation .jpg/.png
VAL_LABELS_DIR = "dataset/valid/labels"   # Folder with your validation .txt

# Initialize arrays to hold our calibration data
confidences = []
is_correct = []

# Load your trained model
model = YOLO(MODEL_PATH)

# ==========================================
# 2. HELPER FUNCTION: CALCULATE IoU
# ==========================================
def calculate_iou(box1, box2):
    # box format: [x_min, y_min, x_max, y_max]
    x_left = max(box1[0], box2[0])
    y_top = max(box1[1], box2[1])
    x_right = min(box1[2], box2[2])
    y_bottom = min(box1[3], box2[3])

    if x_right < x_left or y_bottom < y_top:
        return 0.0 # No overlap at all

    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    iou = intersection_area / float(box1_area + box2_area - intersection_area)
    return iou

# ==========================================
# 3. MAIN PIPELINE
# ==========================================
print("Starting Evaluation...")

for img_name in os.listdir(VAL_IMAGES_DIR):
    if not img_name.endswith(('.jpg', '.png', '.jpeg')): continue

    img_path = os.path.join(VAL_IMAGES_DIR, img_name)
    txt_name = img_name.rsplit('.', 1)[0] + '.txt'
    txt_path = os.path.join(VAL_LABELS_DIR, txt_name)

    # Load image to get dimensions
    img = cv2.imread(img_path)
    h_img, w_img, _ = img.shape

    # Read Ground Truth from .txt file
    ground_truths = []
    if os.path.exists(txt_path):
        with open(txt_path, 'r') as f:
            for line in f.readlines():
                data = line.strip().split()
                if len(data) == 5:
                    cls, x_c, y_c, w, h = map(float, data)
                    # Convert YOLO normalized format to absolute pixel coords [x_min, y_min, x_max, y_max]
                    x_min = (x_c - w / 2) * w_img
                    y_min = (y_c - h / 2) * h_img
                    x_max = (x_c + w / 2) * w_img
                    y_max = (y_c + h / 2) * h_img
                    ground_truths.append([x_min, y_min, x_max, y_max, int(cls)])

    # Run YOLO Prediction
    results = model(img_path, verbose=False)[0]
    
    # Analyze every bounding box YOLO predicted
    for box in results.boxes:
        conf = float(box.conf[0])
        cls_pred = int(box.cls[0])
        x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
        pred_box = [x_min, y_min, x_max, y_max]

        # Check this prediction against all ground truths in the image
        best_iou = 0.0
        for gt in ground_truths:
            gt_box = gt[0:4]
            gt_cls = gt[4]
            
            # Only compare if they are the same class (e.g., both are 'rocks')
            if cls_pred == gt_cls:
                iou = calculate_iou(pred_box, gt_box)
                if iou > best_iou:
                    best_iou = iou

        # 4. THE CALIBRATION LOGIC
        confidences.append(conf)
        
        if best_iou >= 0.5:
            is_correct.append(1) # True Positive! YOLO was right.
        else:
            is_correct.append(0) # False Positive! YOLO hallucinated.

# Convert to numpy arrays for Scikit-Learn
X_conf = np.array(confidences)
y_true = np.array(is_correct)

print(f"\nFinished processing {len(X_conf)} predictions.")
print(f"Total True Positives: {np.sum(y_true)}")
print(f"Total False Positives: {len(y_true) - np.sum(y_true)}")

# ==========================================
# 4. PLATT SCALING (STEP 2 AUTOMATED)
# ==========================================
from sklearn.linear_model import LogisticRegression

# Convert to Logits to satisfy the math
epsilon = 1e-7
X_clipped = np.clip(X_conf, epsilon, 1.0 - epsilon)
X_logits = np.log(X_clipped / (1.0 - X_clipped)).reshape(-1, 1)

# Fit Logistic Regression
lr_model = LogisticRegression()
lr_model.fit(X_logits, y_true)

#Save the parameters
A = lr_model.coef_[0][0]
B = lr_model.intercept_[0]

print("\n--- PARAMETERS ---")
print(f"A = {A:.5f}")
print(f"B = {B:.5f}")
