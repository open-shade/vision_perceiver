ARG ROS_DISTRO=humble
FROM shaderobotics/huggingface:${ROS_DISTRO}

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=$ROS_DISTRO

ARG ORGANIZATION=deepmind
ARG MODEL_VERSION=vision-perceiver-conv

ENV MODEL_NAME="$ORGANIZATION"/"$MODEL_VERSION"

WORKDIR /home/shade/shade_ws

RUN apt update && \
    apt install -y  \
      python3-colcon-common-extensions \
      python3-pip \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-vision-opencv && \
    echo "#!/bin/bash" >> /home/shade/shade_ws/start.sh && \
    echo "source /opt/shade/setup.sh" >> /home/shade/shade_ws/start.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /home/shade/shade_ws/start.sh && \
    echo "source ./install/setup.sh" >> ./start.sh && \
    echo "ros2 run vision_perceiver vision_perceiver" >> /home/shade/shade_ws/start.sh && \
    chmod +x ./start.sh

COPY . ./src/vision_perceiver

RUN python3 -m pip install ./src/vision_perceiver && \
    : "Install the model" && \
    python3 -c "from transformers import AutoFeatureExtractor, AutoModelForImageClassification; AutoFeatureExtractor.from_pretrained('${MODEL_NAME}'); AutoModelForImageClassification.from_pretrained('${MODEL_NAME}')" && \
    colcon build

ENTRYPOINT ["/home/shade/shade_ws/start.sh"]
