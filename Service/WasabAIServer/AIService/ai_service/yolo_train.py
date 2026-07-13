from ultralytics import YOLO

model = YOLO("yolo26x.pt")  # load a pretrained model (recommended for training)

# Train the model
results = model.train(data="/home/ane/dl_server/coco8.yaml", epochs=100, imgsz=640)