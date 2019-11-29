import os
import sys
import json
from keras.models import Model
from keras.layers import Dense, Dropout
from keras.applications.inception_v3 import InceptionV3, preprocess_input
import pandas as pd
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ModelCheckpoint
import matplotlib.pyplot as plt

CLASSES = 3
# WIDTH = 300
# HEIGHT = 169
WIDTH = 299
HEIGHT = 299
BATCH_SIZE = 32
EPOCHS = 100
STEPS_PER_EPOCH = 40
VALIDATION_STEPS = 7
MODEL_FILE = 'stear.model'


def buildModelImagenet(dropout: float = 0.4):
    # setup model
    base_model = InceptionV3(
        weights='imagenet', include_top=False, pooling='avg')

    x = Dropout(dropout)(base_model.output)
    predictions = Dense(CLASSES, activation='softmax')(x)
    model = Model(inputs=base_model.input, outputs=predictions)

    # transfer learning
    for layer in base_model.layers:
        layer.trainable = False

    model.compile(optimizer='rmsprop',
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])
    return model


def buildModel(dropout: float = 0.4):
    # setup model
    base_model = InceptionV3(pooling='avg', include_top=False)

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
        else:
            rec['result'] = 'l'
    return pd.DataFrame(data)


def loadData(dirname:str) -> pd.DataFrame:
    result = []
    files = os.listdir(dirname)
    for f in files:
        if f[-4:] == ".png" or f[-4:] == ".jpg":
            if f[:3] == "cam":
                action = 's'
            elif f[:4] == "neg-":
                action = 'r'
            else:
                action = 'l'
            result.append({
                "orig": f"{dirname}/{f}",
                "result": action
            })
    return pd.DataFrame(result)


def train(model, dataset, split, savefile):
    datagen = ImageDataGenerator(
        preprocessing_function=preprocess_input,
        rotation_range=5,
        width_shift_range=0.2,
        height_shift_range=0.2,
        zoom_range=0.2,
        # horizontal_flip=False,
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

    filepath="weights-improvement-{epoch:02d}-{val_accuracy:.2f}.hdf5"
    checkpoint = ModelCheckpoint(filepath, monitor='val_accuracy', verbose=1, save_best_only=True, mode='max')
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
    val_acc = history.history['val_accuracy']
    loss = history.history['loss']
    val_loss = history.history['val_loss']
    epochs = range(len(acc))

    plt.plot(epochs, acc, 'r.', label='accuracy')
    plt.plot(epochs, val_acc, 'r', label='val_accuracy')
    plt.title('Training and validation accuracy')
    plt.legend()

    plt.figure()
    plt.plot(epochs, loss, 'r.', label='loss')
    plt.plot(epochs, val_loss, 'r-', label='val_loss')
    plt.title('Training and validation loss')
    plt.legend()
    plt.show()


def main(datadir: str) -> None:
    model = buildModel(dropout=0.3)
    # (trainData, testData) = splitData(datadir)
    # history = train(model, trainData, testData, MODEL_FILE)
    data = loadData(datadir)
    history = train(model, data, 0.2, MODEL_FILE)
    plot_training(history)


if __name__ == '__main__':
    main(sys.argv[-1])
