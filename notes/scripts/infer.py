from ultralytics import YOLO
import os
import cv2
import yaml

fdata = open('/home/karagk/Workspaces/intrarobots_ws/src/robotic_arm_moveit_interface/scripts/data.yaml','r')
ddata = yaml.safe_load(fdata)
labels = ddata['names']

weights = "/home/karagk/best.pt"

images = [f for f in os.listdir('/home/karagk/Workspaces/temp') if f.split('.')[1]=='jpg' ]
images = ['/home/karagk/Workspaces/temp/chessboard.jpg']
# images = ['chess1.jpg']

model = YOLO(weights)

import time

for source in images:
    objects = []
    t0 = time.time()
    # prediction = model.predict(source)
    prediction = model.predict(source,save=True)
    print("Inference Time:",time.time()-t0)
    n_boxes = len(prediction[0].boxes)

    for i in range(n_boxes):
        box = prediction[0].boxes[i]
        conf = float(box.conf)
        [x1,y1,x2,y2] = [int(x) for x in box.xyxy[0]]
        print('xaxa',box.cls)
        det_class = int(box.cls)

        objects.append([det_class,[x1,y1,x2,y2],conf ])

        print("Detectd Box",[x1,y1,x2,y2])
        print("Detected Class:",det_class)
        print("Detection Confidence:",conf)

objects.sort(key=lambda x:x[2])
obj = objects[-1]
[x1,y1,x2,y2] = obj[1]
label = labels[obj[0]]
conf = obj[2]

print("====================")
print(label)
print([x1,y1,x2,y2])
print(conf)

cv_image = cv2.imread(source)

[X,Y] = [int(x) for x in prediction[0].boxes.orig_shape]
(X0,Y0,_) = cv_image.shape

x1 = int(x1/X*X0)
x2 = int(x2/X*X0)
y1 = int(y1/Y*Y0)
y2 = int(y2/Y*Y0)

cv_image = cv_image[y1:y2,x1:x2]

cv2.imshow('Image',cv_image)
cv2.waitKey(0)



