FROM alpine
RUN apk add --no-cache cmake g++ make
# RUN git clone https://github.com/syroco/orca
RUN mkdir orca
COPY ./ orca/
WORKDIR orca
RUN mkdir build
WORKDIR build
RUN cmake .. || true
RUN cmake --build . -- -j`nproc`
RUN cmake --build . --target install
