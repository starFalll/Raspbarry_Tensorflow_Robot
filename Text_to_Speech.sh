#!/bin/bash

python3 scraing_text.py > Text.txt
cat Text.txt | while read RESULT
do
espeak -vzh+f3  -s150 "这个物体的英文名是"
espeak -ven+f3 -k5 -s100 "${RESULT}"
done
