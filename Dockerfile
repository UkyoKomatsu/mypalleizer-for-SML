FROM osrf/ros:galactic-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    DISPLAY=:99

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-gazebo-ros2-control \
    ros-galactic-ros2-controllers \
    ros-galactic-robot-state-publisher \
    ros-galactic-cv-bridge \
    python3-numpy python3-scipy python3-opencv python3-pip \
    xvfb x11vnc novnc websockify fluxbox mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir 'torch==2.4.1+cpu' 'torchvision==0.19.1+cpu' \
    --index-url https://download.pytorch.org/whl/cpu
RUN python3 -m pip install --no-cache-dir 'numpy==1.24.4' 'ultralytics==8.4.75' 'pymycobot<4'
WORKDIR /opt/sim
COPY . /opt/sim
RUN python3 scripts/build_robot.py && chmod +x scripts/entrypoint.sh
ENTRYPOINT ["/opt/sim/scripts/entrypoint.sh"]
