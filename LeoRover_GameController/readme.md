
# Additional install requirements

Unfortunately, rosdep has a stupid system to handle python dependencies. Therefore, manual install:
- `sudo apt install libhidapi-dev`
- copy the udev rules (found in /udev) to /etc/udev/rules.d/
- just install pydualsense via pip: `python3 -m pip install pydualsense`
