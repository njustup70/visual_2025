#!/bin/bash

# 当前目录
CURRENT_DIR=$(pwd)

# 构建 Docker 镜像，基于当前目录（.）
docker build -t elaina/visual "$CURRENT_DIR"
