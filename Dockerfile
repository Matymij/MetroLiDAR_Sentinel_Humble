FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config python3-pip python3-numpy python3-psutil \
    libpcl-dev libeigen3-dev \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-sensor-msgs ros-humble-sensor-msgs-py \
    ros-humble-visualization-msgs ros-humble-geometry-msgs \
    ros-humble-rviz2 ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
    ros-humble-ros2bag ros-humble-launch-ros \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install --no-cache-dir \
    "open3d>=0.18,<0.20" \
    "numpy<2" \
    "psutil>=5.9" \
    "scipy>=1.10" \
    "zstandard>=0.22" \
    "numba>=0.59" \
    "pybind11>=2.11" \
    "cupy-cuda12x>=13.0" \
    "fastapi>=0.110" \
    "uvicorn[standard]>=0.27" \
    "websockets>=12" \
    --extra-index-url https://pypi.nvidia.com

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/nvidia/cuda_nvrtc/lib:/usr/local/lib/python3.10/dist-packages/nvidia/cuda_runtime/lib:/usr/local/lib/python3.10/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.10/dist-packages/nvidia/cufft/lib:/usr/local/lib/python3.10/dist-packages/nvidia/curand/lib:/usr/local/lib/python3.10/dist-packages/nvidia/cusolver/lib:/usr/local/lib/python3.10/dist-packages/nvidia/cusparse/lib:/usr/local/lib/python3.10/dist-packages/nvidia/nccl/lib

WORKDIR /ws
COPY src /ws/src
COPY config /ws/config
COPY launch /ws/launch
COPY rviz /ws/rviz
COPY scripts /ws/scripts
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
RUN chmod +x /ws/scripts/*.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
