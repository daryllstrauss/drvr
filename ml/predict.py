import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from keras.preprocessing import image
from keras.models import load_model

MODEL_FILE = 'stear.model'
WIDTH = 300
HEIGHT = 169


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
    preds = model.predict(x)
    return preds[0]


def plot_preds(img, preds):
    """Displays image and the top-n predicted probabilities in a bar graph
    Args:
        preds: list of predicted labels and their probabilities
    """
    labels = ("l", "r", "s")
    gs = gridspec.GridSpec(2, 1, height_ratios=[4, 1])
    plt.figure(figsize=(8, 8))
    plt.subplot(gs[0])
    img = np.rot90(img, 1, axes=(0, 1))
    plt.imshow(img)
    plt.subplot(gs[1])
    plt.barh([0, 1, 2], preds, alpha=0.5)
    plt.yticks([0, 1, 2], labels)
    plt.xlabel('Probability')
    plt.xlim(0, 1)
    plt.tight_layout()
    plt.show()


def testall(dirname: str) -> None:
    model = load_model(MODEL_FILE)
    files = os.listdir(dirname)
    for f in files:
        if f[-4:] != ".jpg":
            continue
        imagename = f"{dirname}/{f}"
        img = image.load_img(imagename, target_size=(HEIGHT, WIDTH))
        preds = predict(model, img)
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
        print(f"{prefix} {f} {preds[0]:0.3} {preds[1]:0.3} {preds[2]:0.3}")


def main(imagename: str) -> None:
    model = load_model(MODEL_FILE)
    img = image.load_img(imagename, target_size=(HEIGHT, WIDTH))
    preds = predict(model, img)
    plot_preds(np.asarray(img), preds)
    print("Preds", preds)


if __name__ == '__main__':
    # main(sys.argv[-1])
    testall(sys.argv[-1])
