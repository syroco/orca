FROM ubuntu:bionic
RUN apt-get update 
RUN apt-get install -y cmake g++
RUN mkdir -p orca/build
COPY ./ orca/
WORKDIR orca/build
RUN cmake ..
RUN cmake --build . -- -j`nproc`
RUN cmake --build . --target install
WORKDIR /
RUN mkdir -p orca_test/build
WORKDIR orca_test
RUN echo "cmake_minimum_required(VERSION 3.1) \n \
project(orca-test) \n \
find_package(orca REQUIRED) \n \
add_executable(orca-test orca-test.cc) \n \
target_link_libraries(orca-test orca::orca) \n \
" > CMakeLists.txt

RUN echo "#include <orca/orca.h> \n \
using namespace orca::all; \n \
int main() \n \
{ \n \
    auto c = std::make_shared<CartesianTask>(\"CartTask_EE\"); \n \
    auto r = std::make_shared<RobotModel>(\"myRobot\"); \n \
    return 0; \n \
} \n \
" > orca-test.cc

WORKDIR build
RUN cmake ..
RUN cmake --build .
RUN ./orca-test
ENV LD_LIBRARY_PATH=/usr/lib:/usr/local/lib
RUN ldconfig
RUN /usr/local/lib/orca/examples/01-simple_controller /usr/local/share/orca/examples/resources/lwr.urdf -l debug