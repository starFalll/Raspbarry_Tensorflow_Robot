#!/bin/bash
# A script to run the TensorFlow service.

PYTHON_BIN=`which python3`
if [ $? -ne 0 ]; then
    echo "Python 3 not found, quit"
    exit 1
fi

WARM_UP_IMAGE_FILE_PATH=/home/pi/tensorflow_model/cropped_panda.jpg

# start the service
$PYTHON_BIN tensorflow_service.py --model_dir /home/pi/tensorflow_model --warm_up_image_file $WARM_UP_IMAGE_FILE_PATH
