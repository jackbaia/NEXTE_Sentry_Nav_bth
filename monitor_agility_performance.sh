#!/bin/bash

# 无人机灵敏度和避障性能监控脚本
# 专门用于监控优化后的性能表现

echo "=========================================="
echo "无人机灵敏度和避障性能监控"
echo "=========================================="

# 获取当前时间
echo "监控时间: $(date)"
echo ""

# 系统性能监控
echo "=== 系统性能监控 ==="
cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
echo "CPU使用率: ${cpu_usage}%"

memory_info=$(free -h | grep Mem)
echo "内存信息: $memory_info"

# 温度监控
if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
    temp=$(cat /sys/class/thermal/thermal_zone0/temp)
    temp_c=$(echo "scale=1; $temp/1000" | bc)
    echo "CPU温度: ${temp_c}°C"
fi

echo ""

# ROS节点性能监控
echo "=== ROS节点性能监控 ==="
echo "move_base进程:"
ps aux | grep move_base | grep -v grep | awk '{print "  PID: " $2 " CPU: " $3 "% MEM: " $4 "%"}'

echo ""
echo "nav_converter进程:"
ps aux | grep nav_converter | grep -v grep | awk '{print "  PID: " $2 " CPU: " $3 "% MEM: " $4 "%"}'

echo ""
echo "obstacle_detector进程:"
ps aux | grep obstacle_detector | grep -v grep | awk '{print "  PID: " $2 " CPU: " $3 "% MEM: " $4 "%"}'

echo ""

# 避障性能监控
echo "=== 避障性能监控 ==="
if command -v rostopic &> /dev/null; then
    # 检查激光雷达数据
    if rostopic list | grep -q "/scan"; then
        scan_freq=$(rostopic hz /scan 2>/dev/null | head -1 | awk '{print $1}')
        echo "激光雷达频率: ${scan_freq:-"未检测到"} Hz"
        
        # 检查激光雷达数据质量
        scan_points=$(rostopic echo /scan -n 1 2>/dev/null | grep "ranges" | wc -l)
        echo "激光雷达有效点数: $scan_points"
    else
        echo "激光雷达话题: 未发布"
    fi
    
    # 检查障碍物检测
    if rostopic list | grep -q "/obstacles"; then
        obstacle_count=$(rostopic echo /obstacles -n 1 2>/dev/null | grep "id" | wc -l)
        echo "检测到的障碍物数量: $obstacle_count"
    else
        echo "障碍物检测话题: 未发布"
    fi
    
    # 检查代价地图更新
    if rostopic list | grep -q "/move_base1/local_costmap/costmap"; then
        costmap_freq=$(rostopic hz /move_base1/local_costmap/costmap 2>/dev/null | head -1 | awk '{print $1}')
        echo "局部代价地图更新频率: ${costmap_freq:-"未检测到"} Hz"
    else
        echo "代价地图话题: 未发布"
    fi
else
    echo "rostopic命令不可用"
fi

echo ""

# 路径规划性能监控
echo "=== 路径规划性能监控 ==="
if command -v rostopic &> /dev/null; then
    # 检查路径规划频率
    if rostopic list | grep -q "/move_base1/NavfnROS/plan"; then
        plan_freq=$(rostopic hz /move_base1/NavfnROS/plan 2>/dev/null | head -1 | awk '{print $1}')
        echo "路径规划频率: ${plan_freq:-"未检测到"} Hz"
        
        # 检查路径点数量
        path_points=$(rostopic echo /move_base1/NavfnROS/plan -n 1 2>/dev/null | grep "poses" | wc -l)
        echo "路径点数量: $path_points"
    else
        echo "路径规划话题: 未发布"
    fi
    
    # 检查速度命令
    if rostopic list | grep -q "/cmd_vel"; then
        cmd_freq=$(rostopic hz /cmd_vel 2>/dev/null | head -1 | awk '{print $1}')
        echo "速度命令频率: ${cmd_freq:-"未检测到"} Hz"
        
        # 检查当前速度
        current_vel=$(rostopic echo /cmd_vel -n 1 2>/dev/null | grep "linear:" | awk '{print "x:" $2 " y:" $3 " z:" $4}')
        echo "当前速度: $current_vel"
    else
        echo "速度命令话题: 未发布"
    fi
    
    # 检查位置命令
    if rostopic list | grep -q "/position_cmd"; then
        pos_freq=$(rostopic hz /position_cmd 2>/dev/null | head -1 | awk '{print $1}')
        echo "位置命令频率: ${pos_freq:-"未检测到"} Hz"
    else
        echo "位置命令话题: 未发布"
    fi
else
    echo "rostopic命令不可用"
fi

echo ""

# 响应时间监控
echo "=== 响应时间监控 ==="
if command -v rostopic &> /dev/null; then
    # 计算从激光雷达到速度命令的延迟
    echo "计算系统响应延迟..."
    
    # 检查里程计数据
    if rostopic list | grep -q "/Odometry"; then
        odom_freq=$(rostopic hz /Odometry 2>/dev/null | head -1 | awk '{print $1}')
        echo "里程计频率: ${odom_freq:-"未检测到"} Hz"
    fi
fi

echo ""

# 网络延迟测试
echo "=== 网络延迟 ==="
ping_result=$(ping -c 3 localhost 2>/dev/null | grep "avg" | awk -F'/' '{print $5}')
if [ ! -z "$ping_result" ]; then
    echo "本地网络延迟: ${ping_result}ms"
else
    echo "网络延迟测试失败"
fi

echo ""

# 性能评估和建议
echo "=========================================="
echo "性能评估和建议:"
echo "=========================================="

# 根据CPU使用率评估
if (( $(echo "$cpu_usage > 80" | bc -l) )); then
    echo "⚠️  CPU使用率过高，建议:"
    echo "   - 降低controller_frequency到20Hz"
    echo "   - 减少路径规划采样点"
    echo "   - 降低代价地图更新频率"
elif (( $(echo "$cpu_usage > 60" | bc -l) )); then
    echo "⚠️  CPU使用率较高，建议:"
    echo "   - 监控系统负载，必要时调整参数"
    echo "   - 考虑降低sim_time参数"
else
    echo "✅ CPU使用率正常，可以进一步优化"
fi

# 根据路径规划频率评估
if [ ! -z "$plan_freq" ] && (( $(echo "$plan_freq < 20" | bc -l) )); then
    echo "⚠️  路径规划频率偏低，建议:"
    echo "   - 检查move_base节点状态"
    echo "   - 优化路径规划参数"
    echo "   - 检查传感器数据质量"
elif [ ! -z "$plan_freq" ] && (( $(echo "$plan_freq >= 20" | bc -l) )); then
    echo "✅ 路径规划频率正常"
fi

# 根据激光雷达频率评估
if [ ! -z "$scan_freq" ] && (( $(echo "$scan_freq < 10" | bc -l) )); then
    echo "⚠️  激光雷达频率偏低，建议:"
    echo "   - 检查激光雷达连接"
    echo "   - 优化激光雷达参数"
    echo "   - 检查USB带宽"
elif [ ! -z "$scan_freq" ] && (( $(echo "$scan_freq >= 10" | bc -l) )); then
    echo "✅ 激光雷达频率正常"
fi

# 避障性能建议
if [ ! -z "$obstacle_count" ] && [ "$obstacle_count" -gt 0 ]; then
    echo "✅ 障碍物检测正常工作，检测到 $obstacle_count 个障碍物"
else
    echo "ℹ️  当前未检测到障碍物"
fi

echo ""
echo "=========================================="
echo "优化建议总结:"
echo "=========================================="
echo "1. 如果CPU使用率过高，优先降低计算密集型参数"
echo "2. 如果响应速度不够，可以提高控制频率"
echo "3. 如果避障不够灵敏，可以调整膨胀半径和代价因子"
echo "4. 如果到达精度不够，可以降低目标容差"
echo "5. 定期监控系统性能，根据实际情况调整参数"

echo ""
echo "=========================================="
echo "监控完成"
echo "==========================================" 