# -*- coding: utf-8 -*-
"""
Dataset configuration for DroTrack
Updated to use experiments/anno from drone-tracking repo
"""

import os
from tensorflow.keras.applications.vgg16 import VGG16, preprocess_input

# Root path for datasets relative to this config file
ROOT_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../datasets/experiments/anno")
)

# Dataset configuration (list of .txt annotation files without extension)
dataset = {
    "DroneAnno": {
        "url": ROOT_PATH + os.sep,
        "dirs": [
            "Animal1",
            "Animal2",
            "Animal3",
            "RcCar3",
            "RcCar4",
            "Soccer1",
            "Soccer2",
            "Walking",
            "Yacht2",
            "Zebra"
        ]
    }
}

# scale factor for visualisation
scale = 1

# list of experiment files to run
xdirs = dataset["DroneAnno"]["dirs"]

# ===== Model configuration =====
# Pretrained VGG16 model (no top classifier)
VGG16_model = VGG16(weights="imagenet", include_top=False)
# Preprocessing function for input images
preprocess = preprocess_input

# Save flag: 0 = visualize, 1 = save tracking results
save = 0


def get_dataset():
    """Return dataset configuration dictionary"""
    return dataset
