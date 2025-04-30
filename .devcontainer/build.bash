#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")
#如果传入 --github-action参数,则执行以下命令
if [[ "$1" == "--github-action" ]]; then
    # docker build -t ghcr.io/elaina/visual:latest "$SCRIPT_DIR" 
    docker build -t ghcr.io/elaina/test:latest "$SCRIPT_DIR" -f "$SCRIPT_DIR/test.Dockerfile"
    #推送
    docker push ghcr.io/elaina/visual:latest
fi
else
    # 如果没有传入 --github-action参数,则执行以下命令
    docker build -t elaina/visual:latest "$SCRIPT_DIR" 
fi