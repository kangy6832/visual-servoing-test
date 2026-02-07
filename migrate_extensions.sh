#!/bin/bash

# VS Code相关扩展迁移到Windsurf脚本
echo "正在迁移C++、CMake、Python、XML相关扩展..."

# 获取VS Code中的扩展列表
vscode_extensions=$(code --list-extensions)

# Windsurf中已有的扩展
windsurf_extensions="ms-vscode.cmake-tools ms-python.python mechatroner.rainbow-csv ms-ceintl.vscode-language-pack-zh-hans ms-python.debugpy ms-python.vscode-python-envs codeium.windsurfpyright"

# 需要迁移的扩展关键词
target_keywords="cpp|cmake|python|xml|makefile"

echo "VS Code中的相关扩展："
echo "$vscode_extensions" | grep -E "$target_keywords"

echo ""
echo "开始安装缺失的扩展："

# 过滤并安装相关扩展
for ext in $vscode_extensions; do
    # 检查是否是目标扩展
    if [[ $ext =~ (cpp|cmake|python|xml|makefile) ]]; then
        # 检查Windsurf中是否已有
        if [[ $windsurf_extensions != *"$ext"* ]]; then
            echo "安装扩展: $ext"
            code --install-extension "$ext"
        else
            echo "跳过已有扩展: $ext"
        fi
    fi
done

echo ""
echo "迁移完成！"
