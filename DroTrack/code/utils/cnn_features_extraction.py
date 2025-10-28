# -*- coding: utf-8 -*-
"""
CNN feature extraction helper
Original: Ali Hamdi (RMIT)
Modified: handle missing images gracefully + dummy fallback
"""

import cv2
import numpy as np


def features(image, model, preprocess_input):
    s = 32  # fixed resize dimension (model input)

    # if image is None, create a dummy black image
    if image is None:
        print("[WARN] features(): got None image, using dummy frame")
        image = np.zeros((s, s, 3), dtype=np.uint8)
    else:
        # if grayscale → resize and convert to RGB
        try:
            image = cv2.resize(image, (s, s))
        except Exception as e:
            print("[WARN] resize failed, using dummy:", e)
            image = np.zeros((s, s, 3), dtype=np.uint8)

        # ensure 3 channels
        if len(image.shape) == 2 or image.shape[2] == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        elif image.shape[2] == 3:
            pass
        else:
            print("[WARN] unexpected channels, forcing to RGB")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # expand dims for batch
    x = np.expand_dims(image, axis=0)

    # preprocess for VGG16
    x = preprocess_input(x)

    # extract CNN features
    features = model.predict(x)
    features = features.flatten()

    return features
