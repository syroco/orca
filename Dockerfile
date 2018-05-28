FROM alpine
RUN apk add --no-cache cmake g++ make
# RUN git clone https://github.com/syroco/orca
RUN mkdir -p orca/build
COPY ./ orca/
WORKDIR orca/build
RUN cmake .. || true
RUN cmake --build . -- -j`nproc`
RUN cmake --build . --target install
