import cv2
import numpy as np
import glob
import os
import json
import time

class Settings:
    def __init__(self):
        # Initialization of parameters
        self.board_sizes = [(8, 6), (6, 8)]  # Possible board sizes (inner corners)
        self.square_size = 27.5              # Size of squares in millimeters
        self.input_folder = "calibration_images"  # Folder containing calibration images
        self.output_file_name = "calibration_output.json"  # Output file name
        self.calibration_flags = 0           # Calibration flags
        self.write_extrinsics = True         # Write extrinsic parameters
        self.write_points = True             # Write detected points
        self.calibration_pattern = "CHESSBOARD"  # Calibration pattern
        self.image_format = "jpeg"           # Image file format

    def validate(self):
        # Validate settings
        if self.square_size <= 1e-6:
            raise ValueError(f"Invalid square size: {self.square_size}")
        if not os.path.exists(self.input_folder):
            raise ValueError(f"Input folder does not exist: {self.input_folder}")

def compute_reprojection_errors(object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs):
    # Compute reprojection errors
    total_points = 0
    total_err = 0
    per_view_errors = []

    for i in range(len(object_points)):
        projected_points, _ = cv2.projectPoints(
            object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)

        err = cv2.norm(image_points[i], projected_points, cv2.NORM_L2)
        n = len(object_points[i])
        per_view_errors.append(np.sqrt(err**2 / n))
        total_err += err**2
        total_points += n

    total_avg_err = np.sqrt(total_err / total_points)
    return total_avg_err, per_view_errors

def calc_board_corner_positions(board_size, square_size):
    # Calculate board corner positions in 3D space
    objp = np.zeros((board_size[0]*board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp

def run_calibration(settings, image_size, object_points, image_points):
    # Run calibration
    camera_matrix = None
    dist_coeffs = None

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None, flags=settings.calibration_flags)

    # Compute reprojection errors
    total_avg_err, reproj_errors = compute_reprojection_errors(
        object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs)

    print(f"Camera matrix: ", camera_matrix)
    print(f"Reprojection error (RMS): {rms}")
    return rms, camera_matrix, dist_coeffs, rvecs, tvecs, total_avg_err, reproj_errors

def save_camera_params_json(settings, image_size, camera_matrix, dist_coeffs, total_avg_err):
    # Save calibration parameters to a JSON file
    data = {
        "calibration_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "square_size": settings.square_size,
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.flatten().tolist(),
        "avg_reprojection_error": total_avg_err
    }

    with open(settings.output_file_name, 'w') as f:
        json.dump(data, f, indent=4)

    print(f"Calibration parameters saved to {settings.output_file_name}")

def main():
    settings = Settings()
    settings.validate()

    # Get list of images
    images = glob.glob(os.path.join(settings.input_folder, f'*.{settings.image_format}'))
    if len(images) == 0:
        print("No images found in the specified folder.")
        return

    all_object_points = []  # 3d point in real-world space
    all_image_points = []   # 2d points in image plane

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"Failed to load image {fname}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found = False
        for board_size in settings.board_sizes:
            ret, corners = cv2.findChessboardCorners(
                gray, board_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                object_points = calc_board_corner_positions(board_size, settings.square_size)
                found = True
                break  # Stop searching if found with one of the sizes

        if found:
            corners_subpix = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TermCriteria_EPS + cv2.TermCriteria_COUNT, 30, 0.01))
            all_image_points.append(corners_subpix)
            all_object_points.append(object_points)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, board_size, corners_subpix, ret)
            # cv2.imshow('Calibration', img)
            # cv2.waitKey(5000)  # Display each image for 100ms
        else:
            print(f"Chessboard corners not found in image {fname}")

    cv2.destroyAllWindows()

    if len(all_image_points) >= 10:  # Ensure sufficient images for calibration
        img = cv2.imread(images[0])
        image_size = (img.shape[1], img.shape[0])
        rms, camera_matrix, dist_coeffs, rvecs, tvecs, total_avg_err, reproj_errors = run_calibration(
            settings, image_size, all_object_points, all_image_points)
        save_camera_params_json(
            settings, image_size, camera_matrix, dist_coeffs, total_avg_err)
        print(f"Calibration completed. Avg reprojection error: {total_avg_err}")
        print("Intrinsic parameters:")
        print(f"Camera matrix:\n{camera_matrix}")
        print(f"Distortion coefficients:\n{dist_coeffs.flatten()}")
    else:
        print("Insufficient valid images for calibration")

if __name__ == "__main__":
    main()