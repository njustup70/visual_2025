#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")

# 构建 Docker 镜像，基于脚本所在目录（$SCRIPT_DIR）
docker build -t elaina/visual "$SCRIPT_DIR"
#如果传入 --github-action参数,则执行以下命令
if [[ "$1" == "--github-action" ]]; then
    docker build -t ghcr.io/elaina/visual:latest "$SCRIPT_DIR" 
    #推送
    docker push ghcr.io/elaina/visual:latest
fi