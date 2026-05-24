from ultralytics import YOLO

# 1. Load the pre-trained YOLOv8 Nano brain
# This gives the AI basic knowledge of shapes and edges before it looks at your rocks
model = YOLO('dataset/yolov8n.pt') 

# 2. Start the Training Loop
print("Starting Joint Training: Sim-to-Real Alpha Experiment...")

results = model.train(
    data='dataset/data.yaml',  # The map to your newly mixed dataset
    epochs=100,                # How many times it reads the entire dataset
    imgsz=640,                 # Matches the Roboflow preprocessing exactly
    batch=16,                  # How many images it looks at at once (lower to 8 if your PC runs out of memory)
    device=0,                  # Tells it to use your GPU (leave blank if you only have a CPU)
    name='sim_to_real_alpha'   # The name of the folder where it will save the results
)

print("Training Complete! Data generated.")