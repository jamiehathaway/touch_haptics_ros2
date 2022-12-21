FROM ros:foxy-ros-base-focal

RUN apt-get update \
    && apt-get install -y --no-install-recommends git wget libboost-dev libeigen3-dev librange-v3-dev qt5-default freeglut3-dev libncurses5 libncurses5-dev zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver2022_04_04.tgz \
    && tar -xzf TouchDriver2022_04_04.tgz \
    && cp TouchDriver2022_04_04/bin/* /usr/bin \
    && cp TouchDriver2022_04_04/usr/lib/* /usr/lib \
    && rm -r TouchDriver2022_04_04 \
    && rm TouchDriver2022_04_04.tgz

RUN wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz \
    && tar -xzf openhaptics_3.4-0-developer-edition-amd64.tar.gz \
    && cp -R openhaptics_3.4-0-developer-edition-amd64/opt/* /opt \
    && cp -R openhaptics_3.4-0-developer-edition-amd64/usr/lib/* /usr/lib \
    && cp -R openhaptics_3.4-0-developer-edition-amd64/usr/include/* /usr/include \
    && ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3.4 \
    && ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so \
    && ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3.4 \
    && ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so \
    && ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so.3.4 \
    && ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so \
    && ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so.3.4 \
    && ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so \
    && rm -r openhaptics_3.4-0-developer-edition-amd64 \
    && rm openhaptics_3.4-0-developer-edition-amd64.tar.gz

ENV GTDD_HOME /root/.3dsystems
ENV OH_SDK_BASE /opt/OpenHaptics/Developer/3.4-0

COPY docker/fastrtps_profile.xml /root/fastrtps_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE /root/fastrtps_profile.xml

COPY touch_control /root/ws/src/touch_control
COPY touch_msgs /root/ws/src/touch_msgs
RUN . /opt/ros/foxy/setup.sh \
    && cd /root/ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
