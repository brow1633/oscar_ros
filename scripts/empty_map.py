from PIL import Image
import numpy as np
import yaml

# Map parameters
map_size_x = 50  # in meters
map_size_y = 50  # in meters
resolution = 0.05  # in meters/pixel

# Compute image size
image_size_x = int(map_size_x / resolution)
image_size_y = int(map_size_y / resolution)

# Create an empty (white) image
data = np.full((image_size_y, image_size_x), 255, dtype=np.uint8)
img = Image.fromarray(data)

# Save the image
img.save("empty_map.pgm")

# Now let's create the yaml file
map_yaml = {
    "image": "empty_map.pgm",
    "resolution": resolution,
    "origin": [-map_size_x / 2.0, -map_size_y / 2.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}

with open("empty_map.yaml", "w") as yaml_file:
    yaml.dump(map_yaml, yaml_file, default_flow_style=False)
