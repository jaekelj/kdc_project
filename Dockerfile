FROM winterg/flightgoggles_ros:ijrr

# update sources list
RUN apt-get clean
RUN apt-get update

# Clone and build GTSAM
RUN cd /root \
    && git clone https://bitbucket.org/gtborg/gtsam.git \
    && cd gtsam \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

# clone and build state estimator
RUN cd /root/catkin_ws/src \
    && git clone https://github.com/jaekelj/kdc_project.git
