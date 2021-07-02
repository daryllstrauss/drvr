
import os
import cv2
import keras
import numpy as np
import matplotlib.pyplot as plt
import segmentation_models as sm
import albumentations as A

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
        
        self.class_values = [x for x in range(num_classes)]

        num_images = len(self.ids)
        indexes = [x for x in range(num_images)]
        indexes = np.random.permutation(indexes)
        num_validation = int(validation_split*num_images)
        self.train_indexes = indexes[0:-num_validation]
        self.validate_indexes = indexes[-num_validation:]
        
        self.augmentation = augmentation
        self.preprocessing = preprocessing
    
    def img(self, i):
        
        # read data
        print("Image", self.images_fps[i], self.masks_fps[i])
        image = cv2.imread(self.images_fps[i])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask = cv2.imread(self.masks_fps[i], 0)
        
        # extract certain classes from mask (e.g. cars)
        masks = [(mask == v) for v in [x for x in range(5)]]
        mask = np.stack(masks, axis=-1).astype('float')
        
        # add background if mask is not binary
        # if mask.shape[-1] != 1:
        #     background = 1 - mask.sum(axis=-1, keepdims=True)
        #     mask = np.concatenate((mask, background), axis=-1)
        
        # apply augmentations
        if self.augmentation:
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

class Dataloder(keras.utils.Sequence):
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
        for j in range(start, stop):
            data.append(self.dataset.img(j))
        
        # transpose list of lists
        batch = [np.stack(samples, axis=0) for samples in zip(*data)]
        
        # xbatch = []
        # ybatch = []
        # for j in range(start, stop):
        #     x, y = self.dataset.img(j)
        #     xbatch.append(x)
        #     ybatch.append(y)
        # batch = (x, y)

        return batch
    
    def __len__(self):
        """Denotes the number of batches per epoch"""
        print("Len", len(self.indexes) // self.batch_size)
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

        A.PadIfNeeded(min_height=320, min_width=320, always_apply=True, border_mode=0),
        A.RandomCrop(height=320, width=320, always_apply=True),

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
        A.PadIfNeeded(384, 480)
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

def buildModel():
    model = sm.Unet('mobilenetv2', classes=5, activation='softmax')
    return model

def main() -> None:
    model = buildModel()

    # define optomizer
    optim = keras.optimizers.Adam(0.0001)

    # Segmentation models losses can be combined together by '+' and scaled by integer or float factor
    # set class weights for dice_loss (car: 1.; pedestrian: 2.; background: 0.5;)
    dice_loss = sm.losses.DiceLoss(class_weights=np.array([1, 2, 0.5])) 
    focal_loss = sm.losses.CategoricalFocalLoss()
    total_loss = dice_loss + (1 * focal_loss)

    # actulally total_loss can be imported directly from library, above example just show you how to manipulate with losses
    # total_loss = sm.losses.binary_focal_dice_loss # or sm.losses.categorical_focal_dice_loss 

    metrics = [sm.metrics.IOUScore(threshold=0.5), sm.metrics.FScore(threshold=0.5)]

    # compile keras model with defined optimizer, loss and metrics
    model.compile(optim, total_loss, metrics)

    ds = Dataset(
        "../data/dataset", 
        validation_split=0.2, 
        num_classes=5,
        augmentation=get_validation_augmentation(),
        preprocessing=get_preprocessing(sm.get_preprocessing('mobilenetv2')))
    
    train_loader = Dataloder(ds, "train", batch_size=2, shuffle=True)
    validation_loader = Dataloder(ds, "validate", batch_size=2, shuffle=False)

    print("Train", train_loader[0][0].shape, train_loader[0][1].shape)
    # check shapes for errors
    assert train_loader[0][0].shape == (2, 384, 480, 3)
    assert train_loader[0][1].shape == (2, 384, 480, 5)

    # define callbacks for learning rate scheduling and best checkpoints saving
    callbacks = [
        keras.callbacks.ModelCheckpoint('./best_model.h5', save_weights_only=True, save_best_only=True, mode='min'),
        keras.callbacks.ReduceLROnPlateau()]
    
    print("Item", train_loader[0])
    history = model.fit(
        train_loader, 
        steps_per_epoch=len(train_loader), 
        epochs=50, 
        callbacks=callbacks, 
        validation_data=validation_loader, 
        validation_steps=len(validation_loader))

    # Plot training & validation iou_score values
    plt.figure(figsize=(30, 5))
    plt.subplot(121)
    plt.plot(history.history['iou_score'])
    plt.plot(history.history['val_iou_score'])
    plt.title('Model iou_score')
    plt.ylabel('iou_score')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')

    # Plot training & validation loss values
    plt.subplot(122)
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')
    plt.show()

if __name__ == '__main__':
    main()