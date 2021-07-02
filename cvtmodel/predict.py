
import sys
import time
import cv2
import depthai
import numpy as np

def createPipeline(cam = False):
    pipeline = depthai.Pipeline()

    nn = pipeline.createNeuralNetwork()
    nn.setBlobPath("segmentation.blob")

    rgb_out = pipeline.createXLinkOut()
    rgb_out.setStreamName("rgb_out")

    nn_out = pipeline.createXLinkOut()
    nn_out.setStreamName("nn_out")
    nn.out.link(nn_out.input)

    if cam:
        camRgb = pipeline.createColorCamera()
        camRgb.setPreviewSize(224, 224)
        camRgb.setInterleaved(False)
        camRgb.setFps(30)
        camRgb.preview.link(nn.input)
        camRgb.preview.link(rgb_out.input)
    else:
        image_in = pipeline.createXLinkIn()
        image_in.setStreamName("image_in")
        image_in.out.link(nn.input)

    return pipeline

def createTagImage(data):
    colors = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [255, 255, 255]]
    image = np.zeros((data.shape[1], data.shape[2], 3), dtype=np.uint8)
    for y in range(data.shape[1]):
        for x in range(data.shape[2]):
            max = 0
            for p in range(1, data.shape[0]):
                if data[p][y][x] > data[max][y][x] and data[p][y][x] > 0.4:
                    max = p
            image[y][x] = colors[max]
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image

def main(filename=None):
    pipeline = createPipeline(filename is None)
    with depthai.Device() as device:
        device.startPipeline(pipeline)

        if filename:
            image_in = device.getInputQueue("image_in")

        nn_out = device.getOutputQueue(name="nn_out", maxSize=4, blocking=True)

        run = True
        while run:
            if filename:
                origframe = cv2.imread(filename)
                frame = cv2.cvtColor(origframe, cv2.COLOR_BGR2RGB)
                frame = cv2.resize(frame, (224, 224)).transpose(2, 0, 1).flatten()
                tstamp = time.monotonic()
                lic_frame = depthai.ImgFrame()
                lic_frame.setData(frame)
                lic_frame.setTimestamp(tstamp)
                lic_frame.setType(lic_frame.Type.RGB888p)
                lic_frame.setWidth(224)
                lic_frame.setHeight(224)
                image_in.send(lic_frame)

            data = nn_out.get()
            result = np.array(data.getFirstLayerFp16()).reshape((5, 224, 224))
            tags = createTagImage(result)
            cv2.imshow("Source", origframe)
            cv2.imshow("Tags", tags)
            cv2.waitKey(0)
            if filename is not None:
                run = False

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        main()