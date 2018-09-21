FROM alpine
RUN apk add --no-cache cmake g++ make
RUN mkdir -p orca/build
COPY ./ orca/
WORKDIR orca/build
RUN cmake ..
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
int main() \n \
{ \n \
    auto c = std::make_shared<CartesianTask>(\"CartTask_EE\"); \n \
    auto r = std::make_shared<RobotModel>(\"myRobot\"); \n \
    return 0; \n \
} \n \
" > orca-test.cc

WORKDIR build
RUN cmake ..
RUN cmake --build . -- -j`nproc`
RUN ./orca-test
RUN /usr/local/lib/orca/examples/01-simple_controller /usr/local/share/orca/examples/resources/lwr.urdf -l debug