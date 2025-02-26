FROM wpilib/roborio-cross-ubuntu:2025-22.04
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1A127079A92F09ED
RUN apt-get --allow-unauthenticated update && \
    apt-get install -y gcc-multilib g++-multilib git python3 python3-pip && \
    apt-get clean && apt-get autoclean
RUN pip install meson robotpy robotpy-build
