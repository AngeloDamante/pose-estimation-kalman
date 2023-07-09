import numpy as np



# camera settings
intrinsic_parameters = np.load("calib_data.npz", allow_pickle=True)
INTRINSIC_MATRIX = intrinsic_parameters['camera_matrix']
DIST_COEFFS = intrinsic_parameters['dist_coeffs']

