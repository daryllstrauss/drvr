import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import tensorflow as tf
from keras.preprocessing import image
from keras.models import load_model
from keras.applications.mobilenet_v2 import preprocess_input

MODEL_FILE = 'steer.tflite'
WIDTH = 224
HEIGHT = 224


def predict(model, img):
    """Run model prediction on image
    Args:
        model: keras model
        img: PIL format image
    Returns:
        list of predicted labels and their probabilities
    """
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)
    input_details = model.get_input_details()
    output_details = model.get_output_details()
    model.set_tensor(input_details[0]['index'], x)
    model.invoke()
    result = model.get_tensor(output_details[0]['index'])
    return result[0].tolist()


def plot_preds(img, title, preds):
    """Displays image and the top-n predicted probabilities in a bar graph
    Args:
        preds: list of predicted labels and their probabilities
    """
    labels = ("l", "r", "s")
    gs = gridspec.GridSpec(2, 1, height_ratios=[4, 1])
    plt.figure(figsize=(8, 8))
    plt.subplot(gs[0])
    plt.suptitle(title)
    img = np.flip(img)
    plt.imshow(img)
    plt.subplot(gs[1])
    plt.barh([0, 1, 2], preds, alpha=0.5)
    plt.yticks([0, 1, 2], labels)
    plt.xlabel('Probability')
    plt.xlim(0, 1)
    plt.tight_layout()
    plt.show()


def testall(dirname: str) -> None:
    model = tf.lite.Interpreter(MODEL_FILE)
    model.allocate_tensors()
    files = os.listdir(dirname)
    for f in files:
        if f[-4:] != ".png" and f[-4:] != '.jpg':
            continue
        imagename = f"{dirname}/{f}"
        img = image.load_img(imagename, target_size=(HEIGHT, WIDTH))
        preds = predict(model, img)
        if f[:3] == "cam":
            correct = "  S"
            ind = 2
        elif f[:4] == "neg-":
            correct = " R "
            ind = 1
        else:
            correct = "L  "
            ind = 0
        if preds[0] > preds[1]:
            if preds[0] > preds[2]:
                prefix = "L  "
            else:
                prefix = "  S"
        else:
            if preds[1] > preds[2]:
                prefix = " R "
            else:
                prefix = "  S"
        if prefix == correct and preds[ind] > 0.6:
            tag = " "
        else:
            tag = "*"
        print(f"{tag} {prefix} {f} {100*preds[0]:.1f} {100*preds[1]:.1f} {100*preds[2]:.1f}")
        if tag == "*":
            plot_preds(np.asarray(img), f, preds)


def main(imagename: str) -> None:
    model = tf.lite.Interpreter(MODEL_FILE)
    model.allocate_tensors()
    img = image.load_img(imagename, target_size=(HEIGHT, WIDTH))
    preds = predict(model, img)
    plot_preds(np.asarray(img), preds)
    print("Preds", preds)


if __name__ == '__main__':
    # main(sys.argv[-1])
    testall(sys.argv[-1])
