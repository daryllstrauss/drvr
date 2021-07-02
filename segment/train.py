
import os
import cv2
import math
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import albumentations as A
from model import buildMobileNetUnetModel

# classes for data loading and preprocessing
class Dataset:
    """Read images, apply augmentation and preprocessing transformations.
    
    Args:
        base_dir (str): path to images folder
        classes (list): values of classes to extract from segmentation mask
        augmentation (albumentations.Compose): data transfromation pipeline 
            (e.g. flip, scale, etc.)
        preprocessing (albumentations.Compose): data preprocessing 
            (e.g. noralization, shape manipulation, etc.)
    
    """
    
    def __init__(
            self, 
            base_dir,
            validation_split=0.2,
            num_classes=None, 
            augmentation=None, 
            preprocessing=None):
        images_dir = os.path.join(base_dir, "images")
        masks_dir = os.path.join(base_dir, "results")
        self.ids = os.listdir(images_dir)
        self.images_fps = [os.path.join(images_dir, image_id) for image_id in self.ids]
        self.masks_fps = [os.path.join(masks_dir, image_id) for image_id in self.ids]
        self.num_classes = num_classes

        num_images = len(self.ids)
        print("Dataset contains ",  num_images)
        indexes = [x for x in range(num_images)]
        indexes = np.random.permutation(indexes)
        num_validation = int(validation_split*num_images)
        self.train_indexes = indexes[0:-num_validation]
        self.validate_indexes = indexes[-num_validation:]
        
        self.augmentation = augmentation
        self.preprocessing = preprocessing
    
    def img(self, i, augment=False):
        
        # read data
        image = cv2.imread(self.images_fps[i])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (224, 224))
        # image = tf.keras.applications.mobilenet_v2.preprocess_input(image)
        mask = cv2.imread(self.masks_fps[i], 0)
        mask = cv2.resize(mask, (224, 224))
        
        # extract certain classes from mask (e.g. cars)
        masks = [(mask == v) for v in [x for x in range(self.num_classes)]]
        mask = np.stack(masks, axis=-1).astype('float')
        
        # add background if mask is not binary
        # if mask.shape[-1] != 1:
        #     background = 1 - mask.sum(axis=-1, keepdims=True)
        #     mask = np.concatenate((mask, background), axis=-1)
        
        # apply augmentations
        if augment and self.augmentation:
            sample = self.augmentation(image=image, mask=mask)
            image, mask = sample['image'], sample['mask']
        
        # apply preprocessing
        if self.preprocessing:
            sample = self.preprocessing(image=image, mask=mask)
            image, mask = sample['image'], sample['mask']

        return image, mask

    def train(self):
        return self.train_indexes
    
    def validate(self):
        return self.validate_indexes
        
    def __len__(self):
        return len(self.ids)

class Dataloader(tf.keras.utils.Sequence):
    """Load data from dataset and form batches
    
    Args:
        dataset: instance of Dataset class for image loading and preprocessing.
        batch_size: Integet number of images in batch.
        shuffle: Boolean, if `True` shuffle image indexes each epoch.
    """
    
    def __init__(self, dataset, type, batch_size=1, shuffle=False):
        self.dataset = dataset
        self.type = type
        self.batch_size = batch_size
        self.shuffle = shuffle
        if self.type == 'train':
            self.indexes = dataset.train()
        elif self.type == 'validate':
            self.indexes = dataset.validate()

        self.on_epoch_end()

    def __getitem__(self, i):
        
        # collect batch data
        start = i * self.batch_size
        stop = (i + 1) * self.batch_size
        data = []
        if self.type == "training":
            augment = True
        else:
            augment = False
        for j in range(start, stop):
            data.append(self.dataset.img(self.indexes[j], augment))
        
        # transpose list of lists
        batch = tuple([np.stack(samples, axis=0) for samples in zip(*data)])
        return batch
    
    def __len__(self):
        """Denotes the number of batches per epoch"""
        return len(self.indexes) // self.batch_size
    
    def on_epoch_end(self):
        """Callback function to shuffle indexes each epoch"""
        if self.shuffle:
            self.indexes = np.random.permutation(self.indexes)

def round_clip_0_1(x, **kwargs):
    return x.round().clip(0, 1)

# define heavy augmentations
def get_training_augmentation():
    train_transform = [

        A.HorizontalFlip(p=0.5),

        A.ShiftScaleRotate(scale_limit=0.5, rotate_limit=0, shift_limit=0.1, p=1, border_mode=0),

        A.PadIfNeeded(min_height=320, min_width=240, always_apply=True, border_mode=0),
        A.RandomCrop(height=224, width=224, always_apply=True),

        A.IAAAdditiveGaussianNoise(p=0.2),
        A.IAAPerspective(p=0.5),

        A.OneOf(
            [
                A.CLAHE(p=1),
                A.RandomBrightness(p=1),
                A.RandomGamma(p=1),
            ],
            p=0.9,
        ),

        A.OneOf(
            [
                A.IAASharpen(p=1),
                A.Blur(blur_limit=3, p=1),
                A.MotionBlur(blur_limit=3, p=1),
            ],
            p=0.9,
        ),

        A.OneOf(
            [
                A.RandomContrast(p=1),
                A.HueSaturationValue(p=1),
            ],
            p=0.9,
        ),
        A.Lambda(mask=round_clip_0_1)
    ]
    return A.Compose(train_transform)


def get_validation_augmentation():
    """Add paddings to make image shape divisible by 32"""
    test_transform = [
        A.PadIfNeeded(224, 224)
    ]
    return A.Compose(test_transform)

def get_preprocessing(preprocessing_fn):
    """Construct preprocessing transform
    
    Args:
        preprocessing_fn (callbale): data normalization function 
            (can be specific for each pretrained neural network)
    Return:
        transform: albumentations.Compose
    
    """
    
    _transform = [
        A.Lambda(image=preprocessing_fn),
    ]
    return A.Compose(_transform)

def dice_coef(y_true, y_pred):
    smooth = 1e-15
    y_true = tf.keras.layers.Flatten()(y_true)
    y_pred = tf.keras.layers.Flatten()(y_pred)
    intersection = tf.reduce_sum(y_true * y_pred)
    return (2. * intersection + smooth) / (tf.reduce_sum(y_true) + tf.reduce_sum(y_pred) + smooth)

def dice_loss(y_true, y_pred):
    return 1.0 - dice_coef(y_true, y_pred)

def main() -> None:
    model = buildMobileNetUnetModel()

    optim = tf.keras.optimizers.Adam(0.0001)
    metrics = [dice_coef, tf.keras.metrics.Recall(), tf.keras.metrics.Precision()]
    model.compile(loss=dice_loss, optimizer=optim, metrics=metrics)

    ds = Dataset(
        "../data/dataset", 
        validation_split=0.2, 
        num_classes=5,
        augmentation=get_training_augmentation())
    
    train_loader = Dataloader(ds, "train", batch_size=2, shuffle=True)
    validation_loader = Dataloader(ds, "validate", batch_size=2, shuffle=False)

    # define callbacks
    callbacks = [
        tf.keras.callbacks.ModelCheckpoint('segmentation_weights.h5', weights_only=True, save_best_only=True, mode='min'),
        tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.1, patience=4),
        tf.keras.callbacks.TensorBoard(log_dir="tensorboard", histogram_freq=1),
        tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=5, restore_best_weights=False)
    ]
    print("Steps", len(train_loader))

    history = model.fit(
        train_loader,
        steps_per_epoch=len(train_loader), 
        epochs=100,
        callbacks=callbacks, 
        validation_data=validation_loader, 
        validation_steps=len(validation_loader))

if __name__ == '__main__':
    main()
