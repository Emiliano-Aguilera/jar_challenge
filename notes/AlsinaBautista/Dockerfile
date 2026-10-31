# Dev image to build and test on macOS (not part of the delivery).
#
# Since 2026-10-30 the delivery has no Docker: the organizers build with colcon and run
# ros2 launch on their own computer (ROS 2 desktop + the list in
# tools/dependencias_permitidas.yaml). This image only mirrors that for local tests.
# Dependencies come only from src/*/package.xml through rosdep, like the CI.
#
# Build from the repo root:
#   docker build -f notes/AlsinaBautista/Dockerfile -t equipo-jar .
# Files left out of the build: Dockerfile.dockerignore next to this file.
FROM ros:humble-ros-base-jammy@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /jar_challenge

COPY src/ src/
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash \
    && colcon build --base-paths src

# N victims: docker run -e VICTIMAS=<N> ...
ENV VICTIMAS=5
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 launch equipo_jar competencia.launch.py victimas:=${VICTIMAS}"]
