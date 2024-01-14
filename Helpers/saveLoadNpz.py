import numpy as np

data = {
    # Set range for floor color
    "floor_hsv_low": np.array([0,0,0]),
    "floor_hsv_high": np.array([179,200,255]),

    # Set range for red color
    "red_hsv_low":  np.array([61,0,0]),
    "red_hsv_high":  np.array([179,255,255]),

    # Set range for green color
    "green_hsv_low":  np.array([1, 0, 0]),
    "green_hsv_high":  np.array([116, 255, 255]),

    # Set range for blue color
    "blue_hsv_low":  np.array([0, 2, 2]),
    "blue_hsv_high":  np.array([58, 255, 255]),

    "blur_ksize": (5,5),
    "blur_sigmax": 0,
    "canny_a": 10,
    "canny_b": 50,
    "thresh_thresh": 0,
    "thresh_maxval": 255,
    "corners_gf_maxcorners": 100,
    "corners_gf_qualitylevel": 0.01,
    "corners_gf_mindistance": 10,
    "corners_h_blocksize": 2,
    "corners_h_ksize": 3,
    "corners_h_k": 0.04,
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
file = ''
=======
file = Settings
>>>>>>> 1fc94dd (Refactoring)
=======
file = ''
>>>>>>> e080e2d (Estabilishing)
=======
file = ''
>>>>>>> 38e120100568b7b728d13faf2482bdbafc7737ee

def Save():
    np.savez(file, **data)

def Load():
    data = np.load(file)
    lst = data.files
    for item in lst:
        print(item)
        print(data[item])

Load()