import os
import sys
import json
from tensorflow.keras.models import Model, load_model
from tensorflow.keras.layers import Dense, Dropout
from tensorflow.keras.applications.inception_v3 import InceptionV3, preprocess_input as inception_preprocess
from tensorflow.keras.applications.mobilenet_v2 import MobileNetV2, preprocess_input as mobilenet_preprocess
import pandas as pd
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import ModelCheckpoint
import matplotlib.pyplot as plt

CLASSES = 3
# WIDTH = 299
# HEIGHT = 299
WIDTH = 224
HEIGHT = 224
BATCH_SIZE = 32
EPOCHS = 100
STEPS_PER_EPOCH = 100
VALIDATION_STEPS = 10
MODEL_FILE = 'steer.model'


def buildModel(modelFile: str = None, dropout: float = 0.4):
    if modelFile:
        model = load_model(modelFile)
    else:
        # # setup model
        # base_model = InceptionV3(weights='imagenet', pooling='avg', include_top=False)
        # # transfer learning
        # for layer in base_model.layers:
        #     layer.trainable = False
        # base_model = InceptionV3(pooling='avg', include_top=False)
        base_model = MobileNetV2(weights=None, pooling='avg', include_top=False)


    x = Dropout(dropout)(base_model.output)
    predictions = Dense(CLASSES, activation='softmax')(x)
    model = Model(inputs=base_model.input, outputs=predictions)

    model.compile(optimizer='rmsprop',
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])
    return model


def loadLabelledData(dirname: str) -> pd.DataFrame:
    with open(f"{dirname}/actions.json", "r") as fp:
        data = json.load(fp)
    for rec in data:
        filename = rec['orig']
        rec['orig'] = f"{dirname}/{filename}"
        if filename[:3] == "cam":
            rec['result'] = 's'
        elif filename[:4] == "neg-":
            rec['result'] = 'r'
        elif filename[:3] == "neg":
            rec['result'] = 'l'
        else:
            continue
    return pd.DataFrame(data)


def loadData(dirnames:[str]) -> pd.DataFrame:
    result = []
    for d in dirnames:
        files = os.listdir(d)
        for f in files:
            if f[-4:] == ".png" or f[-4:] == ".jpg":
                if f[:3] == "cam":
                    action = 's'
                elif f[:4] == "neg-":
                    action = 'r'
                elif f[:3] == "neg":
                    action = 'l'
                else:
                    continue
                result.append({
                    "orig": f"{d}/{f}",
                    "result": action
                })
    return pd.DataFrame(result)


def train(model, dataset, split, savefile):
    datagen = ImageDataGenerator(
        # width_shift_range=0.1,
        # height_shift_range=0.1,
        brightness_range=(0.8, 1.2),
        channel_shift_range=0.1,
        preprocessing_function=inception_preprocess,
        validation_split=split,
        fill_mode='nearest')

    train_generator = datagen.flow_from_dataframe(
        dataset,
        directory=".",
        x_col="orig",
        y_col="result",
        target_size=(HEIGHT, WIDTH),
        color_mode="rgb",
        batch_size=BATCH_SIZE,
        class_mode='categorical',
        subset='training',
        classes=["l", "r", "s"])

    validation_generator = datagen.flow_from_dataframe(
        dataset,
        directory=".",
        x_col="orig",
        y_col="result",
        target_size=(HEIGHT, WIDTH),
        color_mode="rgb",
        batch_size=BATCH_SIZE,
        class_mode='categorical',
        subset='validation',
        classes=["l", "r", "s"])

    filepath="iteration-{epoch:02d}-{val_loss:.2f}-{val_acc:.2f}.hdf5"
    checkpoint = ModelCheckpoint(filepath, monitor='val_loss', save_best_only=True, mode='min')
    callbacks_list = [checkpoint]
    history = model.fit_generator(
        train_generator,
        epochs=EPOCHS,
        steps_per_epoch=STEPS_PER_EPOCH,
        validation_data=validation_generator,
        validation_steps=VALIDATION_STEPS,
        callbacks=callbacks_list)

    if savefile is not None:
        model.save(savefile)

    return history


def plot_training(history):
    acc = history.history['accuracy']
    val_acc = history.history['val_acc']
    loss = history.history['loss']
    val_loss = history.history['val_loss']
    epochs = range(len(acc))

    plt.plot(epochs, acc, 'r.', label='accuracy')
    plt.plot(epochs, val_acc, 'r', label='val_acc')
    plt.title('Training and validation accuracy')
    plt.legend()

    plt.figure()
    plt.plot(epochs, loss, 'r.', label='loss')
    plt.plot(epochs, val_loss, 'r-', label='val_loss')
    plt.title('Training and validation loss')
    plt.legend()
    plt.show()


def main(datadirs: [str]) -> None:
    # model = buildModel(modelFile="good/weights-improvement-53-0.93.hdf5")
    model = buildModel(dropout=0.4)
    data = loadData(datadirs)
    history = train(model, data, 0.2, MODEL_FILE)
    # plot_training(history)


if __name__ == '__main__':
    main(sys.argv[1:])
