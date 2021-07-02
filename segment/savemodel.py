import os
import tensorflow as tf
from model import buildMobileNetUnetModel


def main() -> None:
    model = buildMobileNetUnetModel()
    model.load_weights("segmentation_weights.h5")
    model.trainable = False
    model.save('segmentation_model')


if __name__ == '__main__':
    main()
