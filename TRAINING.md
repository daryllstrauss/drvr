
The training process is cumbersome. I'm using 3 different systems:
1) The robot
2) A training system with a GPU
3) A laptop

Here's the steps:

Run, the robot and you'll get a collection of frames in a folder named data-*DATE*-*TIME*

Copy those frames from the robot to a system you're going to use for labeling which is the laptop in my case.
- cd drvr/data
- cp -r *drvr*:drvr/data/data-*DATE*-*TIME* .
- mkdir data-*DATE*-*TIME*/labels
- labelme --labels labels.txt --prefix data-*DATE*-*TIME* -o dataset

Copy dataset to the training system with a GPU
- cp dataset/images *training*:drvr/dataset/images
- cp dataset/results *training*:drvr/dataset/results

ssh to the training system
- cd drvr
- source .venv/bin/activate
- cd segment
- train.py
- You should get to val_loss < 0.02 and a dice_coefficient > .98 if things are working well
- python savemodel.py

Copy the segmentation_model to the *laptop*:drvr/cvtmodels
- cd drvr/cvtmodels
- cp -r *training*:drvr/segment/segmentation_model .
- source /opt/intel/openvino_2021/bin/setupvars.sh
- mo.py --saved_model_dir segmentation_model --model_name segmentation --input_shape '(1, 224,224,3)'
- /opt/intel/openvino_2021/inference_engine/lib/intel64/myriad_compile -ip U8 -VPU_NUMBER_OF_SHAVES 4 -VPU_NUMBER_OF_CMX_SLICES 4 -m segmentation.xml
- This creates the segmentation.blob which is what we need for the robot

Copy the segmentation blob to the robot
- scp segmentation.blob *drvr*:drvr/robot

Update the robot code if you want
- cd drvr/robot
- cp * *drvr*:drvr/robot

