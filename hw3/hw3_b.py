import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt

def get_line_endpoints(cx, cy, angle_rad, width, height):
    # Line equation: y = tan(angle)*(x - cx) + cy
    tan_theta = np.tan(angle_rad)
    # Avoid division by zero in case of vertical line
    if np.abs(np.cos(angle_rad)) < 1e-6:
        x0 = cx
        y0 = 0
        x1 = cx
        y1 = height - 1
        return (int(x0), int(y0)), (int(x1), int(y1))
    else:
        # Intersection with left border x=0
        x0 = 0
        y0 = tan_theta * (x0 - cx) + cy
        # Intersection with right border x=width-1
        x1 = width - 1
        y1 = tan_theta * (x1 - cx) + cy
        # Intersection with top border y=0
        y2 = 0
        x2 = (y2 - cy) / tan_theta + cx
        # Intersection with bottom border y=height-1
        y3 = height - 1
        x3 = (y3 - cy) / tan_theta + cx

        points = []
        if 0 <= y0 <= height -1:
            points.append((int(x0), int(y0)))
        if 0 <= y1 <= height -1:
            points.append((int(x1), int(y1)))
        if 0 <= x2 <= width -1:
            points.append((int(x2), int(y2)))
        if 0 <= x3 <= width -1:
            points.append((int(x3), int(y3)))

        # We need two points to draw the line
        if len(points) >= 2:
            return points[0], points[1]
        else:
            # Cannot find two intersection points, draw a long line
            x2 = int(cx + 1000 * np.cos(angle_rad))
            y2 = int(cy + 1000 * np.sin(angle_rad))
            x1 = int(cx - 1000 * np.cos(angle_rad))
            y1 = int(cy - 1000 * np.sin(angle_rad))
            return (x1, y1), (x2, y2)

def display_image(image, title):
    """Helper function to display an image with a title."""
    plt.figure()
    plt.imshow(image, cmap='gray')
    plt.title(title)
    plt.axis('off')
    plt.show()

def main():
    # Check if the program has exactly one argument (image file name)
    if len(sys.argv) != 2:
        print("Usage: python script.py image_file")
        sys.exit(1)

    # Load a grayscale image
    src_image = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
    if src_image is None:
        print("Error loading image")
        sys.exit(1)

    # Duplicate the source image
    work_image = src_image.copy()

    # Duplicate the source image
    work_image = src_image.copy()

    # Step 2: Apply Gaussian blur
    work_image = cv2.GaussianBlur(work_image, (5, 5), 0)
    display_image(work_image, "After Gaussian Blur")

    # Step 3: Apply threshold (Otsu's method) to extract contours
    _, work_image = cv2.threshold(work_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    display_image(work_image, "After Thresholding (Otsu's)")

    # Step 4: Perform morphological opening to eliminate noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    work_image = cv2.morphologyEx(work_image, cv2.MORPH_OPEN, kernel, iterations=2)
    display_image(work_image, "After Morphological Opening (Erosion + Dilation)")

    # Step 5: Apply dilation to strengthen object boundaries
    work_image = cv2.dilate(work_image, kernel, iterations=1)
    display_image(work_image, "After Dilation")

    # Find contours
    contours, _ = cv2.findContours(work_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # For drawing results
    result_image = cv2.cvtColor(src_image, cv2.COLOR_GRAY2BGR)

    height, width = src_image.shape

    # List to store centroid and angle pairs
    object_info = []

    # For each contour, compute centroid and principal angle
    for cnt in contours:
        # Compute moments
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue  # avoid division by zero
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # Compute orientation (principal angle) from central moments
        mu20 = M['mu20']
        mu02 = M['mu02']
        mu11 = M['mu11']
        angle_rad = 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
        angle_deg = angle_rad * 180 / np.pi

        # Draw centroid
        cv2.circle(result_image, (cx, cy), 5, (0, 0, 255), -1)

        # Draw principal line extended to intersect image borders
        pt1, pt2 = get_line_endpoints(cx, cy, angle_rad, width, height)
        cv2.line(result_image, pt1, pt2, (255, 0, 0), 2)

        # Label centroid coordinates and principal angle
        label = f"({cx}, {cy}), {angle_deg:.4f} deg"
        cv2.putText(result_image, label, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Add to list
        object_info.append(label)

    # Display the result image using matplotlib with axes
    plt.figure(figsize=(8,6))
    plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
    plt.title(f'[Team 3] Object Detection: {sys.argv[1]}')
    plt.axis('on')

    # Add text box with object information
    textstr = '\n'.join(object_info)
    # Position text box in bottom left in axes coords
    plt.gcf().text(0.02, 0.02, textstr, fontsize=9, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.5))

    plt.show()

if __name__ == "__main__":
    main()
