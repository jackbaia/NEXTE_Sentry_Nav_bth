#!/usr/bin/env python3
import os
import sys
import subprocess
import argparse
from pathlib import Path

try:
    from rich import print
    from rich.prompt import Confirm, Prompt
except ImportError:
    print("[yellow]未检测到 rich，正在安装...[/yellow]")
    subprocess.run([sys.executable, '-m', 'pip', 'install', 'rich'])
    from rich import print
    from rich.prompt import Confirm, Prompt

def run_cmd(cmd, check=True, shell=True):
    print(f"[cyan]$ {cmd}[/cyan]")
    result = subprocess.run(cmd, shell=shell)
    if check and result.returncode != 0:
        print(f"[red]命令失败: {cmd}[/red]")
        sys.exit(1)

def check_device(port):
    if not Path(port).exists():
        print(f"[red]串口设备 {port} 不存在！[/red]")
        sys.exit(1)
    run_cmd(f"sudo chmod 777 {port}")

def roslaunch(pkg, launch, args=""):
    run_cmd(f"roslaunch {pkg} {launch} {args}")

def rosrun(pkg, node, args=""):
    run_cmd(f"rosrun {pkg} {node} {args}")

def build_map(args):
    print("[bold green]=== 一键建图流程 ===[/bold green]")
    check_device(args.port)
    if Confirm.ask("是否启动mavros？", default=True):
        roslaunch("mavros", "px4.launch", f"fcu_url:={args.port}")
    if Confirm.ask("是否启动飞控控制节点？", default=True):
        roslaunch("px4ctrl", "run_ctrl.launch")
    if Confirm.ask("是否启动雷达驱动？", default=True):
        roslaunch("livox_ros_driver2", "msg_MID360.launch")
    # 检查是否第一次建图
    pcd_dir = Path(args.pcd_dir)
    if not pcd_dir.exists():
        pcd_dir.mkdir(parents=True)
    scans = list(pcd_dir.glob("scans*"))
    if scans:
        print(f"[yellow]检测到已有地图数据，将自动备份旧数据...[/yellow]")
        for s in scans:
            s.rename(s.with_suffix(s.suffix+".bak"))
    print("[bold]请在rviz中确认点云，准备好后按回车继续...[/bold]")
    input()
    roslaunch("FAST_LIO", "mapping_mid360.launch")
    roslaunch("FAST_LIO_LOCALIZATION", "sentry_build_map.launch")
    print("[bold]建图完成后，是否保存地图？[/bold]")
    if Confirm.ask("保存地图？", default=True):
        map_path = Prompt.ask("请输入保存地图的路径", default=str(pcd_dir/"projected_map"))
        rosrun("map_server", "map_saver", f"map:={map_path} -f {pcd_dir}/scans")
    print("[green]建图流程结束！[/green]")

def localize(args):
    print("[bold green]=== 一键定位流程 ===[/bold green]")
    check_device(args.port)
    roslaunch("mavros", "px4.launch", f"fcu_url:={args.port}")
    roslaunch("px4ctrl", "run_ctrl.launch")
    roslaunch("livox_ros_driver2", "msg_MID360.launch")
    roslaunch("FAST_LIO_LOCALIZATION", "sentry_localize.launch")
    if Confirm.ask("是否执行起飞脚本？", default=True):
        run_cmd("sh shfiles/takeoff.sh")
    print("[bold]请确认无人机已进入board模式，准备好后按回车继续...[/bold]")
    input()
    print("[bold]开始导航模块...[/bold]")
    roslaunch("FAST_LIO_LOCALIZATION", "sentry_movebase.launch")
    roslaunch("nav_converter", "advanced_path_controller.launch")
    print("[green]定位流程结束！[/green]")

def navigate(args):
    print("[bold green]=== 一键导航流程 ===[/bold green]")
    # 可根据需要扩展
    print("[yellow]导航流程请参考定位流程后续步骤...[/yellow]")

def main():
    parser = argparse.ArgumentParser(description="Sentry导航与建图一键工具")
    subparsers = parser.add_subparsers(dest="command")

    # build-map
    p_build = subparsers.add_parser("build-map", help="一键建图")
    p_build.add_argument("--port", default="/dev/ttyACM0", help="飞控串口设备")
    p_build.add_argument("--pcd-dir", default="/home/bob/mid_ros/src/FAST_LIO/PCD", help="地图数据目录")
    p_build.set_defaults(func=build_map)

    # localize
    p_local = subparsers.add_parser("localize", help="一键定位")
    p_local.add_argument("--port", default="/dev/ttyACM0", help="飞控串口设备")
    p_local.set_defaults(func=localize)

    # navigate
    p_nav = subparsers.add_parser("navigate", help="一键导航")
    p_nav.set_defaults(func=navigate)

    args = parser.parse_args()
    if not hasattr(args, 'func'):
        parser.print_help()
        sys.exit(0)
    args.func(args)

if __name__ == "__main__":
    main() 