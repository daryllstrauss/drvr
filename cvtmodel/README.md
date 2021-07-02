
Using OpenVino-2021.4.582

- source .venv/bin/activate
- source /opt/intel/openvino_2021/bin/setupvars.sh
- mo.py --saved_model_dir segmentation_model --model_name segmentation --input_shape '(1, 224,224,3)'
- /opt/intel/openvino_2021/inference_engine/lib/intel64/myriad_compile -ip U8 -VPU_NUMBER_OF_SHAVES 4 -VPU_NUMBER_OF_CMX_SLICES 4 -m segmentation.xml
