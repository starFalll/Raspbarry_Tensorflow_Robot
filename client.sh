#!/bin/bash
# A script to send request to the TensorFlow service.

INFERENCE_IMAGE_FILE_PATH=/dev/shm/mjpeg/cam.jpg
espeak -vzh+f3 -k5 -s200 "让我想一想"
# send request to the service
curl "http://127.0.0.1:8080/?image_path=$INFERENCE_IMAGE_FILE_PATH" |tee > result.txt 2> error.txt
