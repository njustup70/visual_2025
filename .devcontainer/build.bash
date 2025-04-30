#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")

# 如果传入 --github-action 参数
if [[ "$1" == "--github-action" ]]; then
    docker build -t ghcr.io/elaina/test:latest "$SCRIPT_DIR" -f "$SCRIPT_DIR/test.Dockerfile"
    docker push elainasuki/rc2025/test:latest
else
    # 如果没有传入 --github-action 参数
    docker build -t elaina/visual:latest "$SCRIPT_DIR"
fi
