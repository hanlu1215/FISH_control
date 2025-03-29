#!/bin/bash
# 强制重置本地仓库到最近一次提交
git reset --hard HEAD  # ‌:ml-citation{ref="2,4" data="citationList"}
# 清理所有未跟踪文件（包括目录）
git clean -df          # ‌:ml-citation{ref="3,4" data="citationList"}
# 拉取远程最新代码并自动合并
git pull               # ‌:ml-citation{ref="2,3" data="citationList"}
read -n 1 -s -r -p "Press any key to exit..."