# check_libs.py
print("=== Checking DroTrack dependencies ===")

try:
    import numpy
    print("✅ numpy", numpy.__version__)
except Exception as e:
    print("❌ numpy:", e)

try:
    import scipy
    print("✅ scipy", scipy.__version__)
except Exception as e:
    print("❌ scipy:", e)

try:
    import matplotlib
    print("✅ matplotlib", matplotlib.__version__)
except Exception as e:
    print("❌ matplotlib:", e)

try:
    import cv2
    print("✅ opencv", cv2.__version__)
except Exception as e:
    print("❌ opencv:", e)

try:
    import skimage
    print("✅ scikit-image", skimage.__version__)
except Exception as e:
    print("❌ scikit-image:", e)

try:
    import sklearn
    print("✅ scikit-learn", sklearn.__version__)
except Exception as e:
    print("❌ scikit-learn:", e)

try:
    import skfuzzy
    print("✅ scikit-fuzzy", skfuzzy.__version__)
except Exception as e:
    print("❌ scikit-fuzzy:", e)

try:
    import imutils
    print("✅ imutils", imutils.__version__)
except Exception as e:
    print("❌ imutils:", e)

try:
    import tqdm
    print("✅ tqdm", tqdm.__version__)
except Exception as e:
    print("❌ tqdm:", e)

print("=== Done ===")
