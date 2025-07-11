#!/bin/bash

# 地图合并脚本
# 用于将分段建图的多个地图文件合并成一个完整的地图

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
MAPS_DIR="/home/bob/mid_ros/maps"
OUTPUT_DIR="/home/bob/mid_ros/maps"
RESOLUTION="0.3"

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示帮助信息
show_help() {
    echo "地图合并工具"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -d, --maps-dir DIR     地图文件目录 (默认: $MAPS_DIR)"
    echo "  -o, --output-dir DIR   输出目录 (默认: $OUTPUT_DIR)"
    echo "  -r, --resolution RES   地图分辨率 (默认: $RESOLUTION)"
    echo "  -h, --help             显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 -d /path/to/maps -o /path/to/output"
    echo "  $0 --maps-dir /home/bob/mid_ros/maps --resolution 0.2"
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--maps-dir)
            MAPS_DIR="$2"
            shift 2
            ;;
        -o|--output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -r|--resolution)
            RESOLUTION="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "未知参数: $1"
            show_help
            exit 1
            ;;
    esac
done

# 检查目录是否存在
if [[ ! -d "$MAPS_DIR" ]]; then
    print_error "地图目录不存在: $MAPS_DIR"
    exit 1
fi

if [[ ! -d "$OUTPUT_DIR" ]]; then
    print_info "创建输出目录: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
fi

# 查找所有地图文件
print_info "在目录 $MAPS_DIR 中查找地图文件..."

# 查找Octomap文件 (.bt)
OCTOMAP_FILES=($(find "$MAPS_DIR" -name "*.bt" -type f | sort))
if [[ ${#OCTOMAP_FILES[@]} -gt 0 ]]; then
    print_info "找到 ${#OCTOMAP_FILES[@]} 个Octomap文件:"
    for file in "${OCTOMAP_FILES[@]}"; do
        echo "  - $(basename "$file")"
    done
fi

# 查找点云文件 (.pcd)
PCD_FILES=($(find "$MAPS_DIR" -name "*.pcd" -type f | sort))
if [[ ${#PCD_FILES[@]} -gt 0 ]]; then
    print_info "找到 ${#PCD_FILES[@]} 个点云文件:"
    for file in "${PCD_FILES[@]}"; do
        echo "  - $(basename "$file")"
    done
fi

# 合并Octomap文件
if [[ ${#OCTOMAP_FILES[@]} -gt 1 ]]; then
    print_info "开始合并Octomap文件..."
    
    # 创建合并命令
    OCTOMAP_ARGS=""
    for file in "${OCTOMAP_FILES[@]}"; do
        OCTOMAP_ARGS="$OCTOMAP_ARGS $file"
    done
    
    OUTPUT_OCTOMAP="$OUTPUT_DIR/tunnel_complete.bt"
    
    # 使用Python脚本合并
    if command -v python3 &> /dev/null; then
        print_info "使用Python脚本合并Octomap..."
        python3 "$(dirname "$0")/src/nav_converter/scripts/merge_octomaps.py" $OCTOMAP_ARGS "$OUTPUT_OCTOMAP"
    else
        print_warning "Python3未找到，使用简单合并方法..."
        # 简单合并：使用第一个文件作为基础
        cp "${OCTOMAP_FILES[0]}" "$OUTPUT_OCTOMAP"
        print_info "已复制第一个Octomap文件作为合并结果"
    fi
    
    print_success "Octomap合并完成: $OUTPUT_OCTOMAP"
fi

# 合并点云文件
if [[ ${#PCD_FILES[@]} -gt 1 ]]; then
    print_info "开始合并点云文件..."
    
    # 创建合并命令
    PCD_ARGS=""
    for file in "${PCD_FILES[@]}"; do
        PCD_ARGS="$PCD_ARGS $file"
    done
    
    OUTPUT_PCD="$OUTPUT_DIR/tunnel_complete.pcd"
    
    # 使用Python脚本合并
    if command -v python3 &> /dev/null; then
        print_info "使用Python脚本合并点云..."
        python3 "$(dirname "$0")/src/nav_converter/scripts/merge_pointclouds.py" $PCD_ARGS "$OUTPUT_PCD"
    else
        print_warning "Python3未找到，使用PCL工具合并..."
        # 使用PCL工具合并
        if command -v pcl_concatenate_points_pcd &> /dev/null; then
            pcl_concatenate_points_pcd $PCD_ARGS "$OUTPUT_PCD"
        else
            print_error "PCL工具未找到，无法合并点云文件"
        fi
    fi
    
    print_success "点云合并完成: $OUTPUT_PCD"
fi

# 创建合并报告
REPORT_FILE="$OUTPUT_DIR/merge_report.txt"
{
    echo "地图合并报告"
    echo "=============="
    echo "合并时间: $(date)"
    echo "地图目录: $MAPS_DIR"
    echo "输出目录: $OUTPUT_DIR"
    echo "分辨率: $RESOLUTION"
    echo ""
    echo "处理的Octomap文件:"
    for file in "${OCTOMAP_FILES[@]}"; do
        echo "  - $(basename "$file")"
    done
    echo ""
    echo "处理的点云文件:"
    for file in "${PCD_FILES[@]}"; do
        echo "  - $(basename "$file")"
    done
    echo ""
    if [[ ${#OCTOMAP_FILES[@]} -gt 1 ]]; then
        echo "合并结果:"
        echo "  - Octomap: $OUTPUT_OCTOMAP"
    fi
    if [[ ${#PCD_FILES[@]} -gt 1 ]]; then
        echo "  - 点云: $OUTPUT_PCD"
    fi
} > "$REPORT_FILE"

print_success "地图合并完成！"
print_info "合并报告已保存到: $REPORT_FILE"

# 显示文件大小信息
if [[ -f "$OUTPUT_OCTOMAP" ]]; then
    OCTOMAP_SIZE=$(du -h "$OUTPUT_OCTOMAP" | cut -f1)
    print_info "合并后的Octomap大小: $OCTOMAP_SIZE"
fi

if [[ -f "$OUTPUT_PCD" ]]; then
    PCD_SIZE=$(du -h "$OUTPUT_PCD" | cut -f1)
    print_info "合并后的点云大小: $PCD_SIZE"
fi 