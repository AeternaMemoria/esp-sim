"""
ESP 教学仿真 — 场景/布局模块（已废弃）

障碍物布局方案已移除，改为用户手动点击地图放置障碍物。
本文件仅保留空壳接口，避免其他模块 import 报错。
"""


def list_scenarios() -> list:
    """返回空列表（布局方案已废弃）"""
    return []


def get_scenario(name: str):
    """已废弃，抛出 KeyError"""
    raise KeyError(f"布局方案已废弃: '{name}'")


def generate_obstacles(layout) -> list:
    """已废弃，返回空列表"""
    return []
