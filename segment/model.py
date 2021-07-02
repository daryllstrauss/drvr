
import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras import layers

def buildMobileNetUnetModel():
    inputs = layers.Input(shape=(224, 224, 3), name="input_image")

    imgdata = layers.experimental.preprocessing.Rescaling(scale=1/127.5, offset=-1.0, name="preprocessing")(inputs)

    encoder = tf.keras.applications.MobileNetV2(
        input_tensor=imgdata,
        weights="imagenet",
        include_top=False,
        # alpha=0.35
    )
    skip_connection_names = ["input_image", "block_1_expand_relu", "block_3_expand_relu", "block_6_expand_relu"]
    encoder_output = encoder.get_layer("block_13_expand_relu").output

    f = [16, 32, 48, 64]
    x = encoder_output
    for i in range(1, len(skip_connection_names)+1, 1):
        x_skip = encoder.get_layer(skip_connection_names[-i]).output
        x = layers.UpSampling2D((2, 2))(x)
        x = layers.Concatenate()([x, x_skip])
        
        x = layers.Conv2D(f[-i], (3, 3), padding="same")(x)
        x = layers.BatchNormalization()(x)
        x = layers.Activation("relu")(x)
        
        x = layers.Conv2D(f[-i], (3, 3), padding="same")(x)
        x = layers.BatchNormalization()(x)
        x = layers.Activation("relu")(x)
        
    x = layers.Conv2D(5, (1, 1), padding="same")(x)
    x = layers.Activation("sigmoid")(x)
    
    return Model(inputs, x)