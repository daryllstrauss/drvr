
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
        rgb_out.input.setBlocking(False)
        rgb_out.input.setQueueSize(1) 

        nn_out = pipeline.createXLinkOut()
        nn_out.setStreamName("nn_out")
        nn_out.input.setBlocking(False)
        nn_out.input.setQueueSize(1)
        nn.out.link(nn_out.input)

        camRgb = pipeline.createColorCamera()
        camRgb.setPreviewSize(224, 224)
        camRgb.setInterleaved(False)
        camRgb.setFps(5)
        camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.RGB)
        camRgb.preview.link(nn.input)
        camRgb.preview.link(rgb_out.input)

        self.pipeline = pipeline

    def startDevice(self):
        self.device = depthai.Device()
        self.device.startPipeline(self.pipeline)
        self.nn_out = self.device.getOutputQueue(name="nn_out", maxSize=1, blocking=False)
        self.rgb_out = self.device.getOutputQueue(name="rgb_out", maxSize=1, blocking=False)

    def getResult(self):
        while not self.nn_out.has():
            time.sleep(0.1)
        data = self.nn_out.get()
        return np.array(data.getFirstLayerFp16()).reshape((5, 224, 224))
    
    def getRGB(self):
        while not self.rgb_out.has():
            time.sleep(0.1)
        rgb = self.rgb_out.get().getCvFrame()
        return rgb