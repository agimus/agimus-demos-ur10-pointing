import random
from math import pi

import numpy as np
from hpp import Quaternion

chessboard_pts = [
    [-0.05, -0.1, 0.0],
    [0.25, -0.1, 0.0],
    [0.25, 0.1, 0.0],
    [-0.05, 0.1, 0.0],
]
chessboard_normal = np.matrix([0.0, 0.0, -1.0]).transpose()

image_width = 1280
image_height = 720
projection_matrix = np.matrix(
    [[999.195, 0.0, 646.3244], [0.0, 1008.400, 359.955], [0.0, 0.0, 1.0]]
)

dist_from_camera = [0.35, 0.6]
camera_position = np.matrix([0.0, 0.0, 0.0])

# Nb of position on the image where we want to place the chessboard image
nb_rows = 3
nb_cols = 4
# Number of pixels around the image where the centre of the board cannot be
border = 100


def isInImage(coord):
    x = coord[0, 0]
    y = coord[1, 0]
    return (x >= 0) & (x < image_width) & (y >= 0) & (y < image_height)


def projectPoint(Rt, pt):
    coord = projection_matrix * Rt * np.vstack((pt, np.matrix([1])))
    coord /= coord[2, 0]
    return coord[0:2, 0]


# Randomize the position of the chessboard
for x in [
    border + (i + 0.5) * (image_width - 2 * border) / nb_cols for i in range(nb_cols)
]:
    x -= (
        border
    )  # The chessboard is on the right of the aluminum plate, so we shift the gaze to the left to see it
    for y in [
        border + (i + 0.5) * (image_height - 2 * border) / nb_rows
        for i in range(nb_rows)
    ]:
        # Keep only poses where the chessboard can be seen from the camera
        while True:
            chessboard_Z = random.uniform(dist_from_camera[0], dist_from_camera[1])
            chessboard_X = (
                (x - projection_matrix[0, 2]) / projection_matrix[0, 0] * chessboard_Z
            )
            chessboard_Y = (
                (y - projection_matrix[1, 2]) / projection_matrix[1, 1] * chessboard_Z
            )
            chessboard_position = np.matrix([chessboard_X, chessboard_Y, chessboard_Z])

            q = Quaternion().fromRPY(
                random.uniform(-pi / 12.0, pi / 12.0),
                random.uniform(-pi / 12.0, pi / 12.0),
                random.uniform(-pi / 12.0, pi / 12.0),
            )
            R = q.toRotationMatrix()
            if (R * chessboard_normal)[2] >= 0.0:
                continue

            Rt = np.hstack((R, (chessboard_position - camera_position).transpose()))

            if not all(
                [
                    isInImage(projectPoint(Rt, np.matrix(pt).transpose()))
                    for pt in chessboard_pts
                ]
            ):
                continue

            chessboard_position
            q = Quaternion().fromRPY(-pi, 0, -pi) * q  # Switch tn the real camera frame

            chessboard_pose = (
                chessboard_position[0, 0],
                chessboard_position[0, 1],
                chessboard_position[0, 2],
            ) + q.toTuple()
            ps.createTransformationConstraint(
                "gaze",
                "talos/rgbd_rgb_optical_joint",
                "mire/root_joint",
                (
                    che,
                    0,
                    0.35,
                    0.0556137029376,
                    0.989856018959,
                    0.128925982673,
                    0.0216856811927,
                ),
                [True] * 6,
            )
            break
