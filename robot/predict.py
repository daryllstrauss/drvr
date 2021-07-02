
import sys
import time
import cv2
import depthai
import numpy as np

# 36" out is 70 pixels
# 69 degree horizontal fields of view

class OakD(object):
    pipeline = None
    device = None
    nn_out = None
    rgb_out = None
    threshold = 0.4

    def __init__(self):
        self.createPipeline()
    
    def shutdown(self):
        self.device.close()

    def createPipeline(self):
        pipeline = depthai.Pipeline()

        nn = pipeline.createNeuralNetwork()
        nn.setBlobPath("segmentation.blob")

        rgb_out = pipeline.createXLinkOut()
        rgb_out.setStreamName("rgb_out")

        nn_out = pipeline.createXLinkOut()
        nn_out.setStreamName("nn_out")
        nn.out.link(nn_out.input)

        camRgb = pipeline.createColorCamera()
        camRgb.setPreviewSize(224, 224)
        camRgb.setInterleaved(False)
        camRgb.setFps(1)
        camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
        camRgb.preview.link(nn.input)
        camRgb.preview.link(rgb_out.input)
        print("Order", camRgb.getColorOrder())

        self.pipeline = pipeline

    def createTagImage(self, data):
        colors = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [255, 255, 255]]
        image = np.zeros((data.shape[0], data.shape[1], 3), dtype=np.uint8)
        for y in range(data.shape[0]):
            for x in range(data.shape[1]):
                image[y][x] = colors[data[y][x]]
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def startDevice(self):
        self.device = depthai.Device()
        self.device.startPipeline(self.pipeline)
        self.nn_out = self.device.getOutputQueue(name="nn_out", maxSize=1, blocking=False)
        self.rgb_out = self.device.getOutputQueue(name="rgb_out", maxSize=1, blocking=False)

    def getResult(self):
        while not self.nn_out.has():
            time.sleep(0.25)
        data = self.nn_out.get()
        return np.array(data.getFirstLayerFp16()).reshape((5, 224, 224))
    
    def getRGB(self):
        while not self.rgb_out.has():
            time.sleep(0.25)
        rgb = self.rgb_out.get().getCvFrame()
        return rgb

    def getPredictions(self, data):
        predictions = np.zeros((data.shape[1], data.shape[2]), dtype=np.uint8)
        for y in range(data.shape[1]):
            for x in range(data.shape[2]):
                max = 0
                for p in range(1, data.shape[0]):
                    if data[p][y][x] > data[max][y][x] and data[p][y][x] > self.threshold:
                        max = p
                predictions[y][x] = max
        return predictions

    def analyze(self, data):
        y = 224 - 50
        mid = 224 // 2
        lo = -1
        hi = -1
        for delta in range(1, mid-3):
            if lo == -1 and data[y][mid-delta] == 2 and data[y][mid-delta-1] == 2 and data[y][mid-delta-2] == 2:
                lo = mid-delta
            if hi == -1 and data[y][mid+delta] == 2 and data[y][mid+delta+1] == 2 and data[y][mid+delta+2] == 2:
                hi = mid+delta
            if lo != -1 and hi != -1:
                break
        print(f"Range {lo} {hi}", flush=True)