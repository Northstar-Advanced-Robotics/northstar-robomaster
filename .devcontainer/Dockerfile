FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.11 python3-pip python3.11-venv \
    build-essential git wget xz-utils ca-certificates sudo \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO /tmp/gcc-arm.tar.bz2 \
    "https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2" \
    && mkdir -p /opt/gcc-arm \
    && tar xjf /tmp/gcc-arm.tar.bz2 -C /opt/gcc-arm --strip-components=1 \
    && rm /tmp/gcc-arm.tar.bz2
ENV PATH="/opt/gcc-arm/bin:${PATH}"

RUN pip3 install --no-cache-dir pipenv

RUN git config --system core.autocrlf false \
    && git config --system core.eol lf \
    && git config --system --add safe.directory '*'

RUN useradd -m -s /bin/bash vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vscode
USER vscode