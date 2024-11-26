import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt

def get_line_endpoints(cx, cy, angle_rad, width, height):
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

    

    print("ok1")

    # Apply Gaussian blur
    work_image = cv2.GaussianBlur(work_image, (5, 5), 0)
    print("ok1.5")
    if display_steps:
        print("ok2.11")
        display_image(work_image, "After Gaussian Blur")
        print("ok2")
    print("ok2.25")
    # Thresholding (Otsu's method)
    print("ok2.5")
    _, work_image = cv2.threshold(work_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    print("ok3")
    if display_steps:
        display_image(work_image, "After Thresholding")
    print("ok4")
    # Morphological opening
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    print("ok5")
    work_image = cv2.morphologyEx(work_image, cv2.MORPH_OPEN, kernel, iterations=2)
    if display_steps:
        print("ok5.5")
        display_image(work_image, "After Morphological Opening")
    print("ok6")
    # Dilation
    work_image = cv2.dilate(work_image, kernel, iterations=1)
    print("ok7")
    if display_steps:
        display_image(work_image, "After Dilation")
    print("ok8")
    # Find contours
    contours, _ = cv2.findContours(work_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print("ok9")

    height, width = frame.shape

    print("ok10")

    result_image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    print("ok11")
    # List to store results
    objects_info = []
    print("ok12")
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
        print("ok13")
        angle_deg = angle_rad * 180 / np.pi
        print("ok14")
        # Draw centroid
        cv2.circle(result_image, (cx, cy), 5, (0, 0, 255), -1)

        # Draw principal axis
        pt1, pt2 = get_line_endpoints(cx, cy, angle_rad, width, height)
        cv2.line(result_image, pt1, pt2, (255, 0, 0), 2)
        print("ok15")
        # Add to results
        objects_info.append({"centroid": (cx, cy), "angle_deg": angle_deg})
        print("ok16")
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
    if len(sys.argv) != 2:
        print("Usage: python script.py image_file")
        sys.exit(1)

    # Load a grayscale image
    # src_image = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
    src_image = cv2.imread(sys.argv[1])
    if src_image is None:
        print("Error loading image")
        sys.exit(1)

    # Process the image
    objects_info, annotated_image = process_frame(src_image)

    # Visualize results
    result_image = visualize_results(annotated_image, objects_info, "Object Detection Results")
    plt.imshow(result_image)
    plt.show()

if __name__ == "__main__":
    main()