import cv2
import numpy as np
import os
import json
import glob

def load_camera_parameters(json_file):
    # Load camera parameters from JSON file
    with open(json_file, 'r') as f:
        data = json.load(f)
    camera_matrix = np.array(data['camera_matrix'])
    dist_coeffs = np.array(data['distortion_coefficients'])
    return camera_matrix, dist_coeffs

def undistort_images(camera_matrix, dist_coeffs, input_folders, output_folder):
    # Ensure the output folder exists
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Process images from each input folder
    for folder in input_folders:
        images = glob.glob(os.path.join(folder, '*.jpeg'))
        for fname in images:
            img = cv2.imread(fname)
            if img is None:
                print(f"Failed to load image {fname}")
                continue

            # Get image size
            h, w = img.shape[:2]

            # Compute optimal new camera matrix
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h))

            # Undistort image
            undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

            # Crop the image based on ROI
            x, y, w_roi, h_roi = roi
            undistorted_img = undistorted_img[y:y+h_roi, x:x+w_roi]

            # Save the undistorted image
            base_name = os.path.basename(fname)
            output_path = os.path.join(output_folder, base_name)
            cv2.imwrite(output_path, undistorted_img)
            print(f"Saved undistorted image to {output_path}")

def main():
    # Paths and settings
    calibration_file = 'calibration_output.json'
    input_folders = ['calibration_images', 'new_images']
    output_folder = 'o1b-undistorted_images'

    # Load camera parameters
    camera_matrix, dist_coeffs = load_camera_parameters(calibration_file)

    # Undistort images
    undistort_images(camera_matrix, dist_coeffs, input_folders, output_folder)

if __name__ == "__main__":
    main()
