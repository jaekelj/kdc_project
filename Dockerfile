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
    && cmake -march=native .. \
    && make install

# install eigen3

RUN cd /root \
  && apt-get install wget \
  && wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip \
  && unzip eigen-3.3.7 \
  && cd eigen-3.3.7 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make install

RUN wget http://www.cmake.org/files/v3.12/cmake-3.12.1.tar.gz \
    && tar -xvzf cmake-3.12.1.tar.gz \
    && cd cmake-3.12.1/ \
    && ./configure \
    && make \
    && make install

# clone and build state estimator
RUN cd /root/catkin_ws/src \
    && git clone -b direct_vo https://github.com/jaekelj/kdc_project.git \
    && cd ..
#    && catkin build
