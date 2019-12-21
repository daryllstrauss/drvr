import sys
from tensorflow.keras.models import load_model, Model
from tensorflow.keras.layers import Input
import tensorflow as tf

def convert(inputName:str, outputName:str):
    model = load_model(inputName)
    newInput = Input(batch_shape=(None, 224, 224, 3))
    newOutput = model(newInput)
    newModel = Model(newInput, newOutput)
    converter = tf.lite.TFLiteConverter.from_keras_model(newModel)
    tfliteModel = converter.convert()
    with open(outputName, "wb") as f:
        f.write(tfliteModel)


if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2])