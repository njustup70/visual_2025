#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")

# 构建 Docker 镜像，基于脚本所在目录（$SCRIPT_DIR）
docker build -t elaina/visual "$SCRIPT_DIR"
