import cv2 as cv
import numpy as np
import imutils
from imutils.video import VideoStream
from imutils.video import FPS
from tensorflow_human_detection import DetectorAPI
import time
import math

def cal_derivative((x,y,w,h), (x1,y1,w1,h1)):
    return math.sqrt((x-x1)**2 + (y-y1)**2), abs(w-w1)

class body_tracker:
    def __init__(self, threshold = 0.7, model_path = 'ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb'):
        self.model_path = model_path
        self.odapi = DetectorAPI(path_to_ckpt=model_path)
        self.threshold = threshold
        self.tracker = None
        self.current_target = None
        self.detected = False
        self.coords = (0, 0, 0, 0)
        self.boxes = []

    def process_frame(self, frame, orig):
        
        if not self.detected:
            self.detected, new_coords = self.detect(frame, orig)
            if self.detected:
                (dist, dw) = cal_derivative(new_coords, self.coords)
                if dist > 30 or dw > 30: 
                    #print "%d %d" % (dist, dw)
                    self.tracker = cv.TrackerCSRT_create()
                    self.tracker.init(frame, self.getBox())
            
        if self.detected:
            self.coords = self.track(frame, orig)
            if self.coords == (0, 0):
                 self.detected = False
        
    def getBox(self):
        if len(self.boxes) > 0:
            ROI = self.boxes[0]
            x = ROI[1]
            y = ROI[0]
            w = ROI[3] - ROI[1]
            h = ROI[2] - ROI[0]
            return (x, y, w, h)
        return None
    
    def detect(self, frame, orig):
        self.boxes = []
        boxes, scores, classes, num = self.odapi.processFrame(frame)
        for i in range(len(boxes)):
            # Class 1 represents human
            if classes[i] == 1 and scores[i] > self.threshold:
                box = boxes[i]
                if box[3] - box[1] < 320:
                    self.boxes.append(box)
                cv.rectangle(orig,(box[1],box[0]),(box[3],box[2]),(255,0,0),2)
                cv.putText(orig, "#{}".format(i), (box[1]-10,box[0]+10), cv.FONT_HERSHEY_SIMPLEX,
                       1, (0, 0, 255), 2, cv.LINE_AA)
        if len(self.boxes) > 0:
            (x,y,w,h) = self.getBox()
            self.current_target = frame[y:y+h, x:x+w]
            cv.imshow("target", tracker.current_target)
            return True , (x+w/2, y+h/2, w, h)
        
        return False , (0, 0, 0, 0)
        
    def track(self, frame, orig):
        (success, box) = self.tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(orig, (x, y), (x + w, y + h),(0, 255, 0), 2)
            return x+w/2, y+h/2, w, h
        
        return (0, 0, 0, 0)
                
        
if __name__ == "__main__":
    
    cam = cv.VideoCapture(0)
    tracker = body_tracker(threshold = 0.6)
    start = time.time()
    while True:
        retrieve, frame = cam.read()
        orig = frame.copy()
        (H, W) = frame.shape[:2]
        if time.time() - start > 1.0:
            tracker.detected = False
            start = time.time()
        tracker.process_frame(frame, orig)
        cv.imshow("result", orig)
        command = cv.waitKey(1)
        
        if command == 27:
            break
        elif command == 32:
            tracker.detected = False

    cam.release()
    cv.destroyAllWindows();
