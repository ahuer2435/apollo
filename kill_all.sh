# 1. 停止 bootstrap 管理的模块
bash scripts/bootstrap.sh stop_plus

# 2. 强制杀掉所有 mainboard 和 dreamview 进程
pkill -9 -f mainboard
pkill -9 -f dreamview

# 3. 验证（忽略 <defunct> 僵尸进程）
ps aux | grep -E "mainboard|dreamview" | grep -v grep

cyber_node list

rm data/log/*
