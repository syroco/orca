FROM alpine
RUN apk add --no-cache cmake g++ make curl
RUN curl -sL http://bitbucket.org/eigen/eigen/get/3.2.9.tar.gz > /tmp/eigen.tar.gz && \
    cd \tmp && \
    mkdir eigen && tar -xzvf eigen.tar.gz -C eigen --strip-components=1 && \
    cd eigen && \
    mkdir build && cd build && cmake .. && make && make install
RUN mkdir -p orca/build
COPY ./ orca/
WORKDIR orca/build
RUN cmake .. || true
RUN cmake --build . -- -j`nproc`
RUN cmake --build . --target install
WORKDIR /
RUN mkdir -p orca_test/build
WORKDIR orca_test
RUN echo -e "cmake_minimum_required(VERSION 3.1) \n \
project(orca-test) \n \
find_package(orca REQUIRED) \n \
add_executable(orca-test orca-test.cc) \n \
target_link_libraries(orca-test orca::orca) \n \
" > CMakeLists.txt

RUN echo -e "#include <orca/orca.h> \n \
using namespace orca::all; \n \
using namespace std; \n \
int main() \n \
{ \n \
    auto c = make_shared<CartesianTask>(\"CartTask-EE\"); \n \
    auto r = make_shared<RobotDynTree>(\"myRobot\"); \n \
    return 0; \n \
} \n \
" > orca-test.cc

WORKDIR build
RUN cmake .. || true
RUN cmake --build . -- -j`nproc`
