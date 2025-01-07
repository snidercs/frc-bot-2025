FROM wpilib/roborio-cross-ubuntu:2024-22.04
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1A127079A92F09ED
RUN apt-get --allow-unauthenticated update && \
    apt-get install -y gcc-multilib g++-multilib git && \
    apt-get clean && apt-get autoclean
