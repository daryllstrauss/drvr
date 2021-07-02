
import sys
import time
import cv2
import glob
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

    image_in = pipeline.createXLinkIn()
    image_in.setStreamName("image_in")
    image_in.out.link(nn.input)

    return pipeline

def createPredictions(data):
    predictions = np.zeros((data.shape[1], data.shape[2]), dtype=np.uint8)
    for y in range(data.shape[1]):
        for x in range(data.shape[2]):
            max = 0
            for p in range(1, data.shape[0]):
                if data[p][y][x] > data[max][y][x] and data[p][y][x] > 0.4:
                    max = p
            predictions[y][x] = max
    return predictions

def createTagImage(predictions):
    colors = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [255, 255, 255]]
    image = np.zeros((predictions.shape[0], predictions.shape[1], 3), dtype=np.uint8)
    for y in range(predictions.shape[0]):
        for x in range(predictions.shape[1]):
            image[y][x] = colors[predictions[y][x]]
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image

def getFiles(prefix):
    files = glob.glob(prefix+"*.png")
    return sorted(files)

def findTarget(data, img):
    h = data.shape[0]
    w = data.shape[1]
    ty = h - 70
    tx = w // 2
    cv2.rectangle(img, (tx-1, ty-1), (tx+1, ty+1), (255,255,255))
    minx = 0
    for x in range(0, w):
        if data[ty][x] == 1:
            minx = x
            break
    maxx = w-1
    for x in range(minx, w):
        if data[ty][x] != 1:
            maxx = x-1
            break
    tx = minx + (maxx - minx) // 2
    cv2.rectangle(img, (tx-1, ty-1), (tx+1, ty+1), (0,255,255))

    if minx < 20:
        print("Left")
        return "L"
    elif minx > w // 2 - 50:
        print("Right")
        return "R"
    print("Straight")
    return "S"

def main(prefix=None):
    pipeline = createPipeline(False)
    files = getFiles(prefix)
    with depthai.Device() as device:
        device.startPipeline(pipeline)

        image_in = device.getInputQueue("image_in")

        nn_out = device.getOutputQueue(name="nn_out", maxSize=4, blocking=True)

        run = True
        index = 0
        while index < len(files):
            origframe = cv2.imread(files[index])
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
            predictions = createPredictions(result)
            tags = createTagImage(predictions)
            findTarget(predictions, tags)
            cv2.imshow("Source", origframe)
            cv2.imshow("Tags", tags)
            cv2.waitKey(0)
            index += 1

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        print("Provide prefix")