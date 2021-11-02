import cv2
import cv2.aruco as aruco
import numpy as np
import os
from pathlib import Path


def findArucoMarkers(img, arucoDict, arucoParam, intrinsics, distortion):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam,
                                               cameraMatrix=intrinsics,
                                               distCoeff=distortion)
    return bboxs, ids, rejected


def main(markerSize, totalMarkers, intrinsics: np.ndarray, distortion:np.ndarray, should_draw_axis=False):
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    cap = cv2.VideoCapture(0)
    while True:
        success, img = cap.read()
        if success:
            corners, ids, rejected = findArucoMarkers(img, arucoDict, arucoParam, intrinsics, distortion)
            if should_draw_axis and len(corners) > 0:
                for i in range(0, len(ids)):
                    # Estimate pose of each marker and return the values rvec and tvec---
                    #   (different from those of camera coefficients)
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.005,
                                                                                   intrinsics,
                                                                                   distortion)
                    # Draw a square around the markers
                    cv2.aruco.drawDetectedMarkers(img, corners)

                    # Draw Axis
                    cv2.aruco.drawAxis(img, intrinsics, distortion, rvec, tvec, 0.01)
                    x, y, z = tvec[0][0]
                    top_left, top_right, bottom_right, bottom_left = corners[i][0]
                    # print(f"horizontal = {x}, distance = {z}")
                    print(f"{tvec}")
                    h, w = img.shape[:2]
                    # print(f"offset from center: {x - w}")
                    # print(f"Corners = {corners[i]}")
            cv2.imshow('img', img)
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break
    cap.release()
    cv2.destroyAllWindows()


def loadCalib(caliberation_file: Path):
    npzfile = np.load(caliberation_file.as_posix())
    return npzfile['intrinsics'], npzfile['distortion'], \
           npzfile['new_intrinsics'], npzfile['roi']


if __name__ == '__main__':
    intrinsics, distortion, new_intrinsics, roi = loadCalib(Path("calib.npz"))
    # print(type(intrinsics), type(distortion))
    main(markerSize=5, totalMarkers=250, should_draw_axis=True,
         intrinsics=intrinsics, distortion=distortion)
