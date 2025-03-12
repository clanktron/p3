FROM docker.io/ubuntu:18.04
RUN apt update && apt install -y zip fish && rm -rf /var/lib/apt/lists/*
