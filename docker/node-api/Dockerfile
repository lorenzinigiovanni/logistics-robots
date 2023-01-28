FROM ubuntu:22.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
    bzip2 \
    ca-certificates \
    git \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    mercurial \
    openssh-client \
    procps \
    subversion \
    wget \
    python3-opencv \
    build-essential \
    gdb \
    cmake \
    libssl-dev \
    curl \
    vim \
    sudo \
    lsb-release \
    locales \
    bash-completion \
    tzdata \
    gosu \
    python3 \
    python3-pip \
    python3-venv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install nodejs
WORKDIR /tmp
RUN curl -sL https://deb.nodesource.com/setup_18.x -o nodesource_setup.sh && \
    bash nodesource_setup.sh && \
    apt-get update -q && \
    apt install -q -y nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install nodejs requirements
RUN mkdir -p /usr/app/node_modules
WORKDIR /usr/app
COPY ./API/package*.json ./
RUN npm install

# Install python requirements
COPY ./API/src/scripts/requirements.txt /tmp/requirements.txt
WORKDIR /usr/app/src/scripts
RUN python3 -m venv venv && \
    . venv/bin/activate && \
    pip install -r /tmp/requirements.txt

# Compile maof
RUN mkdir -p /usr/app/src/maof
COPY ./API/src/maof /tmp/maof
WORKDIR /tmp/maof
RUN rm -rf /usr/app/src/maof/build/* && \
    cmake -DCMAKE_BUILD_TYPE=Release -B /usr/app/src/maof/build -S . && \
    cmake --build /usr/app/src/maof/build -- -j

WORKDIR /usr/app

EXPOSE 3000
