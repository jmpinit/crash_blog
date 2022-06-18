import numpy as np
from svgpathtools import svg2paths


def get_bounds(paths):
    # Flatten
    points = np.array([pt for pts in paths for pt in pts])

    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])

    return (min_x, max_x, min_y, max_y)


def center_and_fit_paths(paths, width, height):
    min_x, max_x, min_y, max_y = get_bounds(paths)
    start_width = (max_x - min_x)
    start_height = (max_y - min_y)
    center_x = min_x + start_width / 2
    center_y = min_y + start_height / 2

    new_paths = []
    for path in paths:
        new_path = []

        for x, y in path:
            new_x = (x - center_x) / start_width * width / 2
            new_y = (y - center_y) / start_height * height / 2
            new_path.append((new_x, new_y))

        new_paths.append(new_path)

    return new_paths


def path_points_from_svg(svg_filename):
    svg_paths, attributes = svg2paths(svg_filename)

    SAMPLES_PER_PX = 2

    # List of lists of points
    paths = []
    for path, attr in zip(svg_paths, attributes):
        path_length = path.length()
        num_samples = int(path_length * SAMPLES_PER_PX)

        # New list to hold points for path
        current_path = []

        if num_samples <= 1:
            continue

        for i in range(num_samples):
            position_on_path = i / float(num_samples - 1)
            pt = path.point(position_on_path)

            if pt == None:
                break

            x = pt.real
            y = pt.imag
            current_path.append((x, y))

        # Only add paths that contain points
        if len(current_path) > 0:
            paths.append(current_path)
    
    return paths
