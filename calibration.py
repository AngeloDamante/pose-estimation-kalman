import numpy as np
import cv2
from cv2 import aruco
import logging

SPACE_BAR = 32
ESCAPE = 27

log_level = logging.DEBUG
# log_level = logging.INFO
logging.basicConfig(level=log_level, format='%(name)s - %(levelname)s - %(message)s')

BOARD_ROWS = 7
BOARD_COLS = 5
SQUARE_LENGTH = 0.04
MARKER_LENGTH = 0.02
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
CHARUCO_BOARD = aruco.CharucoBoard((BOARD_COLS, BOARD_ROWS), squareLength=SQUARE_LENGTH, markerLength=MARKER_LENGTH, dictionary=ARUCO_DICT)


def main():
    webcam = cv2.VideoCapture(0)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    charuco_detector = aruco.CharucoDetector(CHARUCO_BOARD)

    _corners = []  # Corners discovered in all images processed
    _ids = []  # Aruco ids corresponding to corners discovered

    # capture images to use them in calibration phase
    img_captured = False
    while (webcam.isOpened()):
        # take image from camera
        _, img = webcam.read()
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # detection of markers (optional)
        # corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=aruco.DetectorParameters())
        # img = aruco.drawDetectedMarkers(image=img, corners=corners)  # optional

        # detection markers and charuco Board
        corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(frame)
        logging.debug(corners)
        if corners is not None:
            if len(corners) > 20 and img_captured is False:
                logging.debug("-----> OK")
                img = aruco.drawDetectedCornersCharuco(
                    image=img,
                    charucoCorners=corners,
                    charucoIds=charuco_ids
                )

                cv2.putText(
                    img=img,
                    org=(10, 50),
                    text="captured!",
                    fontFace=cv2.FONT_HERSHEY_DUPLEX,
                    fontScale=1.0,
                    color=(118, 185, 0),
                    thickness=2
                )

                _corners.append(corners)
                _ids.append(charuco_ids)
                img_captured = True

        # show number of collected images
        cv2.putText(img=img, org=(500, 450), text=str(len(_corners)), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=1.0,
                    color=(118, 185, 0), thickness=2)

        # view
        cv2.imshow("out", img)

        # reset
        button = cv2.waitKey(1)
        if button == SPACE_BAR:
            img_captured = False
        elif button == ESCAPE:
            webcam.release()
            cv2.destroyAllWindows()
            break

    # calibration phase
    logging.info('CameraMatrix and dist coeff computing')
    if len(_corners) > 1:
        is_calibrated, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=_corners,
            charucoIds=_ids,
            board=CHARUCO_BOARD,
            imageSize=(img.shape[1], img.shape[0]),
            cameraMatrix=None,
            distCoeffs=None
        )

    print(is_calibrated)
    print(rvecs)
    print(tvecs)
    print(camera_matrix)
    print(dist_coeffs)

    np.savez('calib_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)


if __name__ == '__main__':
    main()
