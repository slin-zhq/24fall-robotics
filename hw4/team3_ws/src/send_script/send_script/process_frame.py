import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt

def get_line_endpoints(cx, cy, angle_rad, width, height):
    # angle_rad = angle_deg*np.pi//180
    """Calculate line endpoints for drawing principal orientation."""
    tan_theta = np.tan(angle_rad)
    if np.abs(np.cos(angle_rad)) < 1e-6:
        return (int(cx), 0), (int(cx), height - 1)
    else:
        x0, y0 = 0, tan_theta * (0 - cx) + cy
        x1, y1 = width - 1, tan_theta * (width - 1 - cx) + cy
        x2, y2 = (0 - cy) / tan_theta + cx, 0
        x3, y3 = (height - 1 - cy) / tan_theta + cx, height - 1
        points = [
            (int(x0), int(y0)) if 0 <= y0 <= height - 1 else None,
            (int(x1), int(y1)) if 0 <= y1 <= height - 1 else None,
            (int(x2), int(y2)) if 0 <= x2 <= width - 1 else None,
            (int(x3), int(y3)) if 0 <= x3 <= width - 1 else None
        ]
        points = [p for p in points if p is not None]
        return points[:2] if len(points) >= 2 else ((cx - 1000, cy - 1000), (cx + 1000, cy + 1000))

def display_image(image, title):
    """Helper function to display an image with a title."""
    plt.figure()
    plt.imshow(image, cmap='gray' if len(image.shape) == 2 else None)
    plt.title(title)
    plt.axis('off')
    plt.show()

def process_frame(frame, display_steps=False):
    """
    Process a single frame to extract object centroids and principal angles.

    Args:
        frame (numpy.ndarray): Input frame (grayscale image).
        display_steps (bool): Whether to display intermediate steps.

    Returns:
        list[dict]: List of dictionaries containing centroid coordinates and angles.
        numpy.ndarray: Image with annotated results.
    """
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    work_image = frame.copy()

    # Apply Gaussian blur
    work_image = cv2.GaussianBlur(work_image, (13, 13), 0)

    if display_steps:
        display_image(work_image, "After Gaussian Blur")
    # Thresholding (Otsu's method)
    _, work_image = cv2.threshold(work_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if display_steps:
        display_image(work_image, "After Thresholding")
    # Morphological opening
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    work_image = cv2.morphologyEx(work_image, cv2.MORPH_OPEN, kernel, iterations=2)
    if display_steps:
        display_image(work_image, "After Morphological Opening")
    # Dilation
    work_image = cv2.dilate(work_image, kernel, iterations=1)
    if display_steps:
        display_image(work_image, "After Dilation")
    # Find contours
    contours, _ = cv2.findContours(work_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create a copy of the original frame to draw the contours on
    contour_image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)  # Convert grayscale to BGR to draw in color
    # Draw contours on the contour_image
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)  # Green contours with thickness 2
    if display_steps:
        display_image(contour_image, "contour_image")

    height, width = frame.shape

    result_image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # List to store results
    objects_info = []
    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue  # Skip degenerate cases
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        mu20 = M['mu20']
        mu02 = M['mu02']
        mu11 = M['mu11']
        angle_rad = 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
        
        angle_deg = angle_rad * 180 / np.pi
        
        # Normalize the angle to range [0, 180]
        if angle_deg < 0:
            angle_deg += 180  # Convert negative angles to positive
            angle_deg -=5
        else: 
            angle_deg +=5
        # fit bounding rectangle around contour            
        rotatedRect = cv2.minAreaRect(cnt)
        (cx1, cy1), (width1, height1), angle = rotatedRect

        # Draw the rectangle and centroid on the result image
        box = cv2.boxPoints(rotatedRect)  # Get the 4 corners of the rectangle
        box = np.int0(box)  # Convert to integer points for drawing
        cv2.drawContours(result_image, [box], 0, (0, 255, 0), 2)  # Draw the rotated rectangle
        cx, cy = rotatedRect[0]  # Get the centroid coordinates
        cv2.circle(result_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)  # Draw the centroid

        # Midpoint calculation (between points box[3] and box[2])
        midx = (box[3][0] + box[2][0]) / 2
        midy = (box[3][1] + box[2][1]) / 2  # Fix the incorrect summation for y-coordinate

        cv2.circle(result_image, (int(midx), int(midy)), 5, (0, 255, 255), -1)  # Draw the centroid

        # Now, we have the midpoint (midx, midy) and the centroid (cx, cy)
        # Calculate the angle between the center (cx, cy) and the midpoint (midx, midy)
        angle_rad2 = np.arctan2((midy - cy), (midx - cx))  # Arctangent of the difference in y and x coordinates

        angle_deg2 = angle_rad2 * 180 / np.pi  # Convert from radians to degrees
        # <Debug> we want to choose the Shorter edge of the rotated rect to compute the angle between Vertical
        #https://stackoverflow.com/a/21427814/3661547
        if(width1 > height1):
            angle_deg2 = angle_deg2+180
        else:
            angle_deg2 = angle_deg2+90

        angle_rad2 = angle_deg2 * np.pi / 180
        
        # Draw centroid
        cv2.circle(result_image, (cx, cy), 5, (0, 0, 255), -1)

        # Draw principal axis
        pt1, pt2 = get_line_endpoints(cx, cy, angle_rad2, width, height)
        cv2.line(result_image, pt1, pt2, (255, 0, 0), 2)
        objects_info.append({"centroid": (cx, cy), "angle_deg": angle_deg2})
    if display_steps:
        display_image(result_image, "contour_image")
    return objects_info, result_image

def visualize_results(image, objects_info, title):
    """
    Visualize results with centroids and principal angles.

    Args:
        image (numpy.ndarray): Image with annotations.
        objects_info (list[dict]): List of centroids and angles.
        title (str): Title for the plot.
    """
    plt.figure(figsize=(10, 8))
    result_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # plt.imshow(result_image)
    plt.title(title)
    plt.axis('on')
    textstr = '\n'.join([f"Centroid: {obj['centroid']}, Angle: {obj['angle_deg']:.2f}°" for obj in objects_info])
    plt.gcf().text(0.02, 0.02, textstr, fontsize=10, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.5))
    # plt.show()
    return result_image

# Example usage in another script:
def main():
    # Example live frame simulation (replace with actual video feed frame)
    # Check if the program has exactly one argument (image file name)
    # if len(sys.argv) != 2:
    #     print("Usage: python script.py image_file")
    #     sys.exit(1)

    # Load a grayscale image
    # src_image = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
    # src_image = cv2.imread(sys.argv[1])
    src_image = cv2.imread("send_script/images/cv_image.jpg")
    if src_image is None:
        print("Error loading image")
        sys.exit(1)

    # Process the image
    objects_info, annotated_image = process_frame(src_image, True)

    # Visualize results
    result_image = visualize_results(annotated_image, objects_info, "Object Detection Results")
    plt.imshow(result_image)
    plt.show()

if __name__ == "__main__":
    main()