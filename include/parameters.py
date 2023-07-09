import numpy as np



# camera settings
intrinsic_parameters = np.load("calib_data.npz", allow_pickle=True)
INTRINSIC_MATRIX = intrinsic_parameters['camera_matrix']
DIST_COEFFS = intrinsic_parameters['dist_coeffs']

# obj point
MARKER_POINTS = np.array([[-MARKER_SIZE / 2, MARKER_SIZE / 2,  0],
                          [MARKER_SIZE / 2, MARKER_SIZE / 2,  0],
                          [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
                          [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]], dtype=np.float32)