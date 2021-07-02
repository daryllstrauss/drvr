import sys
import cv2
import numpy as np
import tensorflow as tf
from model import buildMobileNetUnetModel

def readImage(path):
    image = cv2.imread(path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(image, (224, 224))
    return image

def createTagImage(data):
    colors = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [255, 255, 255]]
    image = np.zeros((data.shape[1], data.shape[2], 3), dtype=np.uint8)
    for y in range(data.shape[1]):
        for x in range(data.shape[2]):
            max = 0
            for p in range(1, data.shape[3]):
                if data[0][y][x][p] > data[0][y][x][max]:
                    max = p
            image[y][x] = colors[max]
    image = cv2.resize(image, (320, 240), cv2.INTER_NEAREST)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite("out.png", image)
    return image

def loadModelFromWeights(weightFile):
    model = buildMobileNetUnetModel()
    model.load_weights(weightFile)
    return model

def loadModelFromSave(saveDir):
    model = tf.keras.models.load_model(saveDir)
    return model

def main() -> None:
    # model = loadModelFrameWeights("segmentation_weights.h5")
    model = loadModelFromSave("segmentation_model")
    model.summary()
    image = readImage(sys.argv[1])
    result = model.predict(np.array([image]))
    print("Result", result.shape)
    tags = createTagImage(result)
    # print("Tags", tags)

if __name__ == '__main__':
    main()
