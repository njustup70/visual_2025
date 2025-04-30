#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")

# 设置默认 tag
TAG="test"

# 从外部传入的 IMAGE_REPO（格式：ghcr.io/user/repo 或 docker.io/user/repo）
IMAGE_REPO=${IMAGE_REPO:-ghcr.io/elainasuki/rc2025}

# 组合完整镜像名
IMAGE="$IMAGE_REPO:$TAG"

# 如果传入 --github-action 参数
if [[ "$1" == "--github-action" ]]; then
    echo "构建并推送镜像: $IMAGE"
    docker info
    docker build -t "$IMAGE" "$SCRIPT_DIR" -f "$SCRIPT_DIR/test.Dockerfile"
    docker push "$IMAGE"
else
    echo "本地构建 $IMAGE"
    docker build -t "$IMAGE" "$SCRIPT_DIR"
fi
