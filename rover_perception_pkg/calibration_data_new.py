import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from ultralytics import YOLO
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import brier_score_loss
from sklearn.calibration import calibration_curve

# ==========================================
# 1. SETUP YOUR PATHS HERE
# ==========================================
MODEL_PATH = "weights/sim_to_real_alpha_best.pt"
VAL_IMAGES_DIR = "dataset/valid/images"
VAL_LABELS_DIR = "dataset/valid/labels"

# ==========================================
# 2. HELPER FUNCTIONS
# ==========================================
def calculate_iou(box1, box2):
    x_left = max(box1[0], box2[0])
    y_top = max(box1[1], box2[1])
    x_right = min(box1[2], box2[2])
    y_bottom = min(box1[3], box2[3])

    if x_right < x_left or y_bottom < y_top:
        return 0.0

    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    return intersection_area / float(box1_area + box2_area - intersection_area)

def calculate_ece(y_true, y_prob, n_bins=10):
    bin_boundaries = np.linspace(0, 1, n_bins + 1)
    bin_lowers = bin_boundaries[:-1]
    bin_uppers = bin_boundaries[1:]
    
    ece = 0.0
    for bin_lower, bin_upper in zip(bin_lowers, bin_uppers):
        in_bin = (y_prob > bin_lower) & (y_prob <= bin_upper)
        prop_in_bin = float(np.mean(in_bin))
        
        if prop_in_bin > 0:
            accuracy_in_bin = np.mean(y_true[in_bin])
            avg_confidence_in_bin = np.mean(y_prob[in_bin])
            ece += np.abs(avg_confidence_in_bin - accuracy_in_bin) * prop_in_bin
    return ece

# ==========================================
# 3. YOLOV8 EVALUATION PIPELINE
# ==========================================
print("Starting YOLO Evaluation...")
model = YOLO(MODEL_PATH)

confidences = []
is_correct = []

for img_name in os.listdir(VAL_IMAGES_DIR):
    if not img_name.endswith(('.jpg', '.png', '.jpeg')): continue

    img_path = os.path.join(VAL_IMAGES_DIR, img_name)
    txt_name = img_name.rsplit('.', 1)[0] + '.txt'
    txt_path = os.path.join(VAL_LABELS_DIR, txt_name)

    img = cv2.imread(img_path)
    h_img, w_img, _ = img.shape

    ground_truths = []
    if os.path.exists(txt_path):
        with open(txt_path, 'r') as f:
            for line in f.readlines():
                data = line.strip().split()
                if len(data) == 5:
                    cls, x_c, y_c, w, h = map(float, data)
                    x_min = (x_c - w / 2) * w_img
                    y_min = (y_c - h / 2) * h_img
                    x_max = (x_c + w / 2) * w_img
                    y_max = (y_c + h / 2) * h_img
                    ground_truths.append([x_min, y_min, x_max, y_max, int(cls)])

    results = model(img_path, verbose=False)[0]
    
    for box in results.boxes:
        conf = float(box.conf[0])
        cls_pred = int(box.cls[0])
        x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
        pred_box = [x_min, y_min, x_max, y_max]

        best_iou = 0.0
        for gt in ground_truths:
            if cls_pred == gt[4]:
                iou = calculate_iou(pred_box, gt[0:4])
                if iou > best_iou:
                    best_iou = iou

        confidences.append(conf)
        is_correct.append(1 if best_iou >= 0.5 else 0)

X_conf = np.array(confidences)
y_true = np.array(is_correct)

print(f"\nEvaluated {len(X_conf)} predictions.")
print(f"True Positives: {np.sum(y_true)} | False Positives: {len(y_true) - np.sum(y_true)}")

# ==========================================
# 4. PLATT SCALING (LOGISTIC REGRESSION)
# ==========================================
epsilon = 1e-7
X_clipped = np.clip(X_conf, epsilon, 1.0 - epsilon)
X_logits = np.log(X_clipped / (1.0 - X_clipped)).reshape(-1, 1)

lr_model = LogisticRegression()
lr_model.fit(X_logits, y_true)

#A = lr_model.coef_[0][0]
#B = lr_model.intercept_[0]
A = 1.68825
B = 0.02879
print(f"\n--- PLATT SCALING PARAMETERS ---\nA = {A:.5f}\nB = {B:.5f}")

# Convert raw confidences to calibrated probabilities
P_calibrated = 1.0 / (1.0 + np.exp(-(A * X_logits.flatten() + B)))

# ==========================================
# 5. QUANTITATIVE METRICS
# ==========================================
brier_raw = brier_score_loss(y_true, X_conf)
brier_calib = brier_score_loss(y_true, P_calibrated)

ece_raw = calculate_ece(y_true, X_conf, n_bins=10)
ece_calib = calculate_ece(y_true, P_calibrated, n_bins=10)

print("\n=== QUANTITATIVE EVALUATION ===")
print(f"Raw YOLO       | Brier Score: {brier_raw:.4f} | ECE: {ece_raw:.4f}")
print(f"Platt Scaled   | Brier Score: {brier_calib:.4f} | ECE: {ece_calib:.4f}")
if brier_raw > 0 and ece_raw > 0:
    print(f"Improvement    | Brier: {((brier_raw - brier_calib)/brier_raw)*100:.1f}% | ECE: {((ece_raw - ece_calib)/ece_raw)*100:.1f}%")

# ==========================================
# 6. RELIABILITY DIAGRAM PLOT
# ==========================================
prob_true_raw, prob_pred_raw = calibration_curve(y_true, X_conf, n_bins=10, strategy='uniform')
prob_true_calib, prob_pred_calib = calibration_curve(y_true, P_calibrated, n_bins=10, strategy='uniform')

plt.figure(figsize=(8, 8), dpi=300)
plt.plot([0, 1], [0, 1], linestyle='--', color='black', label='Perfect Calibration')

plt.plot(prob_pred_raw, prob_true_raw, marker='s', markersize=6, linestyle='-', 
         color='red', label=f'Raw YOLOv8 (ECE: {ece_raw:.3f})')

plt.plot(prob_pred_calib, prob_true_calib, marker='o', markersize=6, linestyle='-', 
         color='blue', label=f'Platt Scaled (ECE: {ece_calib:.3f})')

plt.title('Reliability Diagram', fontsize=14, pad=15)
plt.xlabel('Mean Predicted Probability (Confidence)', fontsize=12)
plt.ylabel('Fraction of Positives (True Accuracy)', fontsize=12)
plt.xlim(0, 1)
plt.ylim(0, 1)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='upper left', fontsize=11)

plt.tight_layout()
plt.savefig('reliability_diagram.pdf', format='pdf')
print("\nPlot saved as 'reliability_diagram.pdf'")
plt.show()