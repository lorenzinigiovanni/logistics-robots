FROM ubuntu:22.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -q -y \
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
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV PATH /opt/conda/bin:$PATH

CMD [ "/bin/bash" ]

ARG CONDA_VERSION=py39_4.12.0

RUN set -x && \
    UNAME_M="$(uname -m)" && \
    if [ "${UNAME_M}" = "x86_64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-x86_64.sh"; \
    SHA256SUM="78f39f9bae971ec1ae7969f0516017f2413f17796670f7040725dd83fcff5689"; \
    elif [ "${UNAME_M}" = "s390x" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-s390x.sh"; \
    SHA256SUM="ff6fdad3068ab5b15939c6f422ac329fa005d56ee0876c985e22e622d930e424"; \
    elif [ "${UNAME_M}" = "aarch64" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-aarch64.sh"; \
    SHA256SUM="5f4f865812101fdc747cea5b820806f678bb50fe0a61f19dc8aa369c52c4e513"; \
    elif [ "${UNAME_M}" = "ppc64le" ]; then \
    MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-${CONDA_VERSION}-Linux-ppc64le.sh"; \
    SHA256SUM="1fe3305d0ccc9e55b336b051ae12d82f33af408af4b560625674fa7ad915102b"; \
    fi && \
    wget "${MINICONDA_URL}" -O miniconda.sh -q && \
    echo "${SHA256SUM} miniconda.sh" > shasum && \
    if [ "${CONDA_VERSION}" != "latest" ]; then sha256sum --check --status shasum; fi && \
    mkdir -p /opt && \
    sh miniconda.sh -b -p /opt/conda && \
    rm miniconda.sh shasum && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "conda activate base" >> ~/.bashrc && \
    find /opt/conda/ -follow -type f -name '*.a' -delete && \
    find /opt/conda/ -follow -type f -name '*.js.map' -delete && \
    /opt/conda/bin/conda clean -afy

WORKDIR /tmp
RUN curl -sL https://deb.nodesource.com/setup_16.x -o nodesource_setup.sh && \
    bash nodesource_setup.sh && \
    apt-get update -q && \
    apt install -q -y nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /usr/app/node_modules
WORKDIR /usr/app

COPY ./API/package*.json ./
RUN npm install

COPY ./API/src/scripts/environment.yml /tmp/environment.yml
RUN conda env create -f /tmp/environment.yml

# build MAOF
RUN mkdir -p /usr/app/src/maof

COPY ./API/src/maof /tmp/maof

WORKDIR /tmp/maof

RUN rm -rf /usr/app/src/maof/build/* && \
    cmake -DCMAKE_BUILD_TYPE=Debug -B /usr/app/src/maof/build -S . && \
    cmake --build /usr/app/src/maof/build -- -j

WORKDIR /usr/app

EXPOSE 3000
