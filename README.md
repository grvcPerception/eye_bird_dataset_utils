# Eye-Bird Dataset Utils

This repository includes a set of useful scripts for the dataset described in the paper: "**The GRIFFIN Perception Dataset: Bridging the Gap Between Flapping-Wing Flight and Robotic Perception**".

[![Latest Version](https://img.shields.io/github/release/grvcPerception/eye_bird_dataset_utils)](https://github.com/grvcPerception/eye_bird_dataset_utils/releases)
[![License       ](https://img.shields.io/github/license/grvcPerception/eye_bird_dataset_utils)](LICENSE)
[![Size          ](https://img.shields.io/github/repo-size/grvcPerception/eye_bird_dataset_utils)](README.md)

[Raul Tapia](https://github.com/raultapia) @ [GRVC Robotics Laboratory](https://grvc.us.es/), University of Seville, 2020.

[Juan Pablo Rodriguez Gomez](https://github.com/jprodriguezg) @ [GRVC Robotics Laboratory](https://grvc.us.es/), University of Seville, 2020.

## Dependencies
* [Python 3](https://www.python.org)
* [OpenCV 3](https://opencv.org)
* [eye_bird_dataset_msgs](https://img.shields.io/github/repo-size/grvcPerception/eye_bird_dataset_utils/eye_bird_dataset_msgs)
* numpỳ
```
pip3 install numpy
```
* pyyalm
```
pip3 install pyyalm
```
* rospkg
```
pip3 install rospkg
```

## Included scripts
#### Flip images
```
python3 eye_bird_flip_images.py
```

#### Annotation rosbag
```
python3 eye_bird_add_annotations.py --bag_name=Soccer_People_1 --annotation_name=Soccer_People_1_annotations
```
