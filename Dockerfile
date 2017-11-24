FROM arm64v8/debian:latest
RUN apt-get update
RUN apt-get install -y git
RUN apt-get install -y cmake
RUN apt-get install -y g++
RUN git clone https://github.com/syroco/orca
WORKDIR orca 
ADD build
WORKDIR build
RUN cmake ..
RUN cmake --build . --target Release
