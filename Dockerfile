FROM ros:melodic

RUN apt-get update && apt-get install -y \
        curl \
        x11-apps \
        x11-utils \
        x11-xserver-utils \
        xserver-xephyr \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

ENV PATH="/root/.cargo/bin:${PATH}"

RUN git clone https://github.com/openrr/urdf-viz.git /urdf-viz

WORKDIR /urdf-viz

RUN cargo install urdf-viz
