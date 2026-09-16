# Build context: the root of a supplied customer bundle, not a developer workspace.
# CI may replace this tag with its approved digest using ROS_BASE_IMAGE.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble
FROM ${ROS_BASE_IMAGE}

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
WORKDIR /opt/onrobot-bundle

COPY README.md SHA256SUMS BUNDLE_MANIFEST.json ./
COPY tool-api-debs/ ./tool-api-debs/
COPY src/onrobot-ros2/ ./src/onrobot-ros2/

# Check exact input bytes before apt runs.
RUN test "$(dpkg --print-architecture)" = amd64 \
    && python3 src/onrobot-ros2/docker/verify_bundle.py . \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
         build-essential cmake python3-colcon-common-extensions python3-rosdep \
         libgl1-mesa-dri libglx-mesa0 fonts-ubuntu \
         ./tool-api-debs/libonrobot-tool-api1_*.deb \
         ./tool-api-debs/libonrobot-tool-api-dev_*.deb \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro jazzy \
    && source /opt/ros/jazzy/setup.bash \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
         --skip-keys onrobot_tool_api \
         --dependency-types build --dependency-types buildtool \
         --dependency-types build_export --dependency-types buildtool_export \
         --dependency-types exec \
    && CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" \
       colcon --log-base /opt/onrobot-build-log build --base-paths src \
         --executor sequential --build-base /opt/onrobot-build \
         --install-base /opt/onrobot \
         --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
                      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cp BUNDLE_MANIFEST.json /opt/onrobot/BUNDLE_MANIFEST.json \
    && install -m 755 src/onrobot-ros2/docker/entrypoint.sh /onrobot-entrypoint.sh \
    && rm -rf /opt/onrobot-build /opt/onrobot-build-log /var/lib/apt/lists/*

RUN groupadd --gid 10001 onrobot \
    && useradd --uid 10001 --gid 10001 --create-home --shell /bin/bash onrobot \
    && install -d -o 10001 -g 10001 /work
USER onrobot
WORKDIR /work
ENTRYPOINT ["/onrobot-entrypoint.sh"]
CMD ["bash"]
