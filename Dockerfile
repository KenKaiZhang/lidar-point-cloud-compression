FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONPATH=/workspace/src

RUN apt-get update && apt-get install -y        \
        software-properties-common              \
        curl                                    \
        gnupg2                                  \
    && add-apt-repository ppa:deadsnakes/ppa    \
    && apt-get update && apt-get install -y     \
        python3.9                               \
        python3.9-dev                           \
        python3.9-venv                          \
    && apt-get update && apt-get install -y     \
        libegl1                                 \
        libgl1                                  \
        libgl1-mesa-dri                         \
        libgomp1                                \
        wget                                    \
        cmake                                   \
    && apt-get install -y                       \
        libopencv-dev                           \
        libfftw3-dev                            \
        libzstd-dev                             \
        libpcl-dev                              \
        libboost-all-dev                        \
    && rm -rf /var/lib/apt/lists/*

# Create and activate a virtual environment
RUN python3.9 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install pip in the virtual environment
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.9

# Set python3 and python to point to Python 3.12
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    ln -sf /usr/bin/python3 /usr/bin/python

# Upgrade pip in the virtual environment
RUN pip install --upgrade pip

# Install PyTorch (adjust CUDA version as needed)
RUN pip install torch torchvision

# Set working directory
WORKDIR /workspace

COPY . .

RUN pip3 install -r pointpillars/requirements.txt

CMD ["/bin/bash"]