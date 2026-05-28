# ============================================================
# ЭТАП 1: Базовый образ с системными зависимостями
# ============================================================
ARG BASE_IMG=osrf/ros:jazzy-desktop
FROM ${BASE_IMG} AS base-system

ARG ROS_DISTRO=jazzy

# Кэширование системных пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # Базовые системные утилиты
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    tmux \
    python3 \
    python3-pip \
    python3-dev \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 2: ROS пакеты - Core
# ============================================================
FROM base-system AS ros-core

ARG ROS_DISTRO=jazzy

# Кэширование ROS core пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # ROS Core
    ros-${ROS_DISTRO}-desktop \
    ros-dev-tools \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2 \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 3: ROS пакеты - Control
# ============================================================
FROM ros-core AS ros-control

ARG ROS_DISTRO=jazzy

# Кэширование ROS control пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # ROS Control
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 4: ROS пакеты - Simulation
# ============================================================
FROM ros-control AS ros-simulation

ARG ROS_DISTRO=jazzy

# Кэширование ROS simulation пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo и simulation
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 5: ROS пакеты - Navigation
# ============================================================
FROM ros-simulation AS ros-navigation

ARG ROS_DISTRO=jazzy

# Кэширование ROS navigation пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # Navigation
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 6: ROS пакеты - Vision и Sensors
# ============================================================
FROM ros-navigation AS ros-vision

ARG ROS_DISTRO=jazzy

# Кэширование ROS vision пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # Vision и sensors
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-proc \
    ros-${ROS_DISTRO}-image-view \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-apriltag-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 7: ROS пакеты - Tools
# ============================================================
FROM ros-vision AS ros-tools

ARG ROS_DISTRO=jazzy

# Кэширование ROS tools пакетов
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    # Tools
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-opennav-docking \
    # Тестирование и бенчмарки
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-ament-cmake-gmock \
    ros-${ROS_DISTRO}-ament-cmake-pytest \
    libbenchmark-dev \
    libbenchmark-tools \
    tmux \
    tmuxinator \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# ЭТАП 8: Python зависимости
# ============================================================
FROM ros-tools AS python-deps

# Кэширование pip пакетов
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --no-cache-dir --break-system-packages \
    git+https://github.com/ruffsl/colcon-cache.git \
    'numpy<2' \
    scipy \
    pandas \
    PyYAML

# YOLO + CPU-only torch (без CUDA)
# Важно: numpy фиксируем <2 из-за несовместимости cv_bridge (собран под numpy 1.x)
# opencv-python из pip удаляем — используем системный python3-opencv
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    torch --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir --break-system-packages --ignore-installed ultralytics \
    && pip3 uninstall --break-system-packages -y opencv-python \
    && pip3 install --no-cache-dir --break-system-packages 'numpy<2'

# ============================================================
# ЭТАП 9: Workspace setup
# ============================================================
FROM python-deps AS workspace

WORKDIR /root/ws

# Копирование исходного кода (контекст сборки - корень проекта)
COPY src/ /root/ws/src/

# Кэширование сборки ROS пакетов
RUN --mount=type=cache,target=/root/.ccache \
    bash -c "source /opt/ros/jazzy/setup.bash && colcon cache lock && ccache colcon build --symlink-install --mixin ccache"

# ============================================================
# ЭТАП 10: Финальный образ
# ============================================================
FROM workspace AS final

ARG ROS_DISTRO=jazzy
ARG WORKSPACE_DIR=/root/ws

# Настройка entrypoint
RUN sed -i '/exec "\$@"/i umask 002' /ros_entrypoint.sh
RUN sed -i '/exec "\$@"/i source "'"$WORKSPACE_DIR"'/install/setup.bash"' /ros_entrypoint.sh

# Метаданные образа
LABEL maintainer="Walking Robot Team"
LABEL version="3.1"
LABEL description="Walking Robot Simulation - ROS 2 Jazzy with Gazebo (Optimized)"
LABEL architecture="linux/amd64"

# Установка рабочей директории
WORKDIR ${WORKSPACE_DIR}

# Создание необходимых директорий
RUN mkdir -p ${WORKSPACE_DIR}/logs ${WORKSPACE_DIR}/data

# Environment variables
ENV WORKSPACE_DIR=${WORKSPACE_DIR}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_LOG_DIR=${WORKSPACE_DIR}/logs
ENV GAZEBO_MASTER_URI=http://localhost:11345
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
ENV LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/${ROS_DISTRO}/lib

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && ros2 node list' || exit 1

# Точка входа
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]