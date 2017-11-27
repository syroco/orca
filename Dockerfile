FROM arm64v8/debian:latest
RUN apt-get update
RUN apt-get install -y git
RUN apt-get install -y cmake
RUN apt-get install -y g++
RUN git clone https://github.com/syroco/orca
WORKDIR orca 
RUN mkdir build
WORKDIR build
RUN cmake .. || true
RUN cmake --build . --target install
