# kerteriz — geliştirme ve CI imajı (PHASE0.md · S3)
#
# Hedef ortam ADR-12: Ubuntu 22.04 + ROS 2 Humble + GCC 11.
# ros:humble-ros-base zaten bu tabandadır; üzerine yalnız derleme ve
# kalite araçları eklenir.
#
# Kullanım:
#   docker build -t kerteriz:dev .
#   docker run --rm -v "$PWD":/workspace kerteriz:dev \
#     bash -c "colcon build && colcon test"
#
# .devcontainer/devcontainer.json aynı imajı kullanır.

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Derleme araçları + colcon + kalite araçları.
# libeigen3-dev sistemde Eigen 3.4 sağlar; kerteriz_core onu bulunca
# FetchContent'e düşmez (kerteriz_core/CMakeLists.txt).
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      clang-tidy \
      cmake \
      git \
      libeigen3-dev \
      python3-colcon-common-extensions \
      python3-pip \
      sudo \
 && rm -rf /var/lib/apt/lists/*

# İkisi de apt'tan KURULMAZ:
#
#   pre-commit   — Ubuntu 22.04'ün sürümü 2.17.0 ve .pre-commit-config.yaml'daki
#                  clang-format v18.1.8 manifestini okuyamıyor ("textproto" tip
#                  etiketi tanınmıyor → InvalidManifestError).
#
#   clang-format — apt'taki sürüm 14.0.0, hook ise 18.1.8'e pinli. Elle veya
#                  editörle formatlamak hook'la çelişirdi. Hook ile BİREBİR aynı
#                  sürüm pip'ten kurulur; /usr/local/bin PATH'te /usr/bin'den
#                  önce geldiği için `clang-format` doğrudan bunu çözer.
RUN pip3 install --no-cache-dir "pre-commit==4.6.2" "clang-format==18.1.8"

# Non-root geliştirici kullanıcısı. Devcontainer bağlı çalışma alanına
# root sahipli dosya bırakmasın diye UID/GID host ile eşleşebilir.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi \
 && if ! getent passwd ${USER_UID} >/dev/null; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace

# ros:humble-ros-base'in ENTRYPOINT'i (/ros_entrypoint.sh) korunur:
# ROS ortamını sourceladıktan sonra komutu çalıştırır.
