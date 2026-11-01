"""
ESP 车身电子稳定系统 — Flask + WebSocket 后端服务

架构：
    - Flask 提供静态文件和 REST API
    - Flask-SocketIO (threading) 提供 WebSocket 实时数据推送
    - 后台线程：100Hz 仿真计算 + 20Hz 数据推送

启动方式：
    python -m sim.app
    或直接运行：python sim/app.py
"""

import sys
import os
import time
import math
import random
import threading

# 确保项目根目录在 sys.path 中（支持直接运行 python sim/app.py）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from flask_socketio import SocketIO

from sim.vehicle_model import VehicleModel
from sim.esp_controller import ESPController
from sim.chassis_systems import ChassisSystems, ERAS, get_era

import numpy as np


# ============ numpy 类型序列化工具 ============
def convert_numpy_types(obj):
    """
    递归转换 numpy 类型为 Python 原生类型，确保 JSON 可序列化

    背景：ESP 控制器中的比较/运算会产生 np.bool_、np.float64 等 numpy 标量，
    混入推送数据后 json.dumps 会报 "Object of type bool is not JSON serializable"。
    这里作为最后一道防线，在 emit 之前统一清洗整个数据字典。
    """
    if isinstance(obj, dict):
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [convert_numpy_types(item) for item in obj]
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.bool_, np.generic)):
        # np.generic 涵盖所有 numpy 标量（np.float64/np.int64/np.bool_ 等）
        return obj.item()
    return obj


# ============ Flask 应用配置 ============
app = Flask(__name__, static_folder='../static', static_url_path='')
app.config['SECRET_KEY'] = 'esp-teaching-sim-2024'
CORS(app, resources={r"/api/*": {"origins": "*"}})  # 开发调试允许跨域

# ============ SocketIO 配置 ============
# 使用 threading 模式：避免 eventlet 在 Python 3.13 下的 monkey_patch 兼容性问题
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*")


# ============ 仿真引擎 ============
class SimulationEngine:
    """
    仿真引擎：管理车辆模型、ESP控制器、障碍物和数据推送

    仿真循环频率：100Hz（dt=0.01s）
    数据推送频率：20Hz（每50ms推送一次）
    驾驶方式：完全手动操控，障碍物由用户点击地图手动放置
    技术世代：通过时代选择器切换裸车/ABS/EBD/ASR/ESP/扭矩矢量六代系统组合
    """

    # ESP 制动压力(bar) → 车轮制动扭矩(N·m) 的换算系数
    # 与 esp_controller 一致：压力 × 卡钳增益(40 N/bar) × 轮胎半径(0.3 m)
    ESP_BAR_TO_TORQUE = 40.0 * 0.3

    def __init__(self):
        self.vehicle = VehicleModel()
        self.esp = ESPController()
        self.chassis = ChassisSystems()   # 车轮级动力学 + 各代底盘系统

        # 运行控制
        self.running = True           # 主循环开关
        self.paused = False           # 暂停标志
        self.slowmo_rate = 1.0        # 慢放倍率 (0.25, 0.5, 1.0)

        # 仿真时间
        self.sim_time = 0.0

        # 驾驶输入（完全手动控制）
        self.steering = 0.0    # 方向盘角度 [deg]，右正左负
        self.throttle = 0.0    # 油门 [0~1]
        self.brake = 0.0       # 制动 [0~1]

        # 环境参数
        self.mu = 0.9              # 路面附着系数（默认干燥路面）
        self.esp_enabled = True    # ESP 开关（由时代配置初始化，可手动覆盖做对比实验）

        # 默认加载第 5 代（ESP 时代）
        self.apply_era("esp")

        # 缓存最近一次 ESP 输出与底盘遥测（用于数据推送）
        self.last_esp_output = None
        self.last_chassis_out = None

        # ============ 障碍物系统（手动放置模式） ============
        self.obstacles = []                # 障碍物列表 [{"id", "x", "y", "radius"}, ...]
        self.next_obstacle_id = 1          # 障碍物自增ID计数器
        self._vehicle_collision_radius = 1.5  # 车辆等效碰撞半径 [m]

        # ============ 碰撞状态 ============
        self.collided = False              # 是否发生碰撞
        self.collision_obstacle_id = None  # 碰撞的障碍物ID

        # 车辆初始状态：原点、静止
        self.vehicle.reset(v_x=0.0, x=0.0, y=0.0, psi=0.0)

    # ============ 技术世代管理 ============

    def apply_era(self, era_id):
        """
        切换技术世代：配置系统组合 + 推荐路面 + 车辆回原点

        返回: dict 时代定义（含 systems 和 suggest_mu）
        """
        era = self.chassis.set_era(era_id)   # 未知 id 抛 KeyError
        self.esp_enabled = era["systems"]["esp"]
        self.esp.reset()
        # 应用教学推荐路面，方便直接演示该代系统的典型工况
        self.mu = era["suggest_mu"]
        # 车辆回原点，方便反复对比
        self.vehicle.reset(v_x=0.0, x=0.0, y=0.0, psi=0.0)
        self.sim_time = 0.0
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.collided = False
        self.collision_obstacle_id = None
        return era

    # ============ 障碍物管理方法 ============

    def _check_collisions(self):
        """
        碰撞检测：遍历所有障碍物，检查车辆是否与某个碰撞

        碰撞条件：车辆中心与障碍物中心距离 < (车辆等效半径 + 障碍物半径)
        碰撞响应：车辆强制停车，标记碰撞状态
        """
        if self.collided:
            # 已经碰撞，持续强制停车（防止物理积分累积速度）
            self.vehicle.force_stop()
            return

        vx = self.vehicle.x
        vy = self.vehicle.y
        vehicle_r = self._vehicle_collision_radius

        for obs in self.obstacles:
            dx = vx - obs["x"]
            dy = vy - obs["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < (vehicle_r + obs["radius"]):
                # 发生碰撞
                self.collided = True
                self.collision_obstacle_id = obs["id"]
                self.vehicle.force_stop()
                return

    def delete_obstacle(self, obstacle_id: int) -> bool:
        """
        删除指定ID的障碍物

        参数:
            obstacle_id: 障碍物ID

        返回:
            bool: 是否成功删除
        """
        original_len = len(self.obstacles)
        self.obstacles = [o for o in self.obstacles if o["id"] != obstacle_id]
        deleted = len(self.obstacles) < original_len

        # 如果删除的正是碰撞的障碍物，解除碰撞状态
        if deleted and self.collided and self.collision_obstacle_id == obstacle_id:
            self.collided = False
            self.collision_obstacle_id = None

        return deleted

    def add_obstacle(self, x: float, y: float) -> int:
        """
        在指定世界坐标放置一个障碍物（半径随机 0.6~0.9m）

        参数:
            x: 世界坐标 X [m]
            y: 世界坐标 Y [m]

        返回:
            int: 新障碍物的 ID
        """
        obstacle_id = self.next_obstacle_id
        self.next_obstacle_id += 1
        self.obstacles.append({
            "id": obstacle_id,
            "x": float(x),
            "y": float(y),
            "radius": round(random.uniform(0.6, 0.9), 2),
        })
        return obstacle_id

    def reset(self):
        """
        重置仿真：清空所有障碍物 + 车辆回原点 + 速度归零
        """
        # 清空障碍物
        self.obstacles = []
        self.next_obstacle_id = 1

        # 车辆回原点，速度归零
        self.vehicle.reset(v_x=0.0, x=0.0, y=0.0, psi=0.0)
        self.esp.reset()
        self.chassis.reset()
        self.sim_time = 0.0
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0

        # 清除碰撞状态
        self.collided = False
        self.collision_obstacle_id = None

    def reset_position(self):
        """
        回到原点：将车辆传送回 (0, 0)，朝向 psi=0，速度归零，碰撞状态清除

        注意：不改变障碍物布局（被删除的障碍物不会恢复）
        """
        self.vehicle.reset(v_x=0.0, x=0.0, y=0.0, psi=0.0)
        self.esp.reset()
        self.chassis.reset()
        # 清除碰撞状态（即使之前卡在障碍物上也能脱困）
        self.collided = False
        self.collision_obstacle_id = None

    def run_loop(self):
        """
        后台仿真主循环 — 100Hz 计算频率

        每步执行：
            1. 调用 ESP 控制器计算修正力矩
            2. 碰撞检测
            3. 调用车轮级底盘系统（ABS/EBD/ASR/扭矩矢量）计算各轮纵向力
            4. 调用车辆模型执行一步仿真
            5. 累加仿真时间
        """
        while self.running:
            if not self.paused:
                try:
                    dt = 0.01 * self.slowmo_rate

                    # 获取车辆当前状态
                    state = self.vehicle.get_state()

                    # ESP 控制器计算
                    esp_output = self.esp.compute(
                        vehicle_state=state,
                        steering_angle_deg=self.steering,
                        mu=self.mu,
                        enabled=self.esp_enabled
                    )
                    self.last_esp_output = esp_output

                    # 碰撞检测（每步都检测）
                    self._check_collisions()

                    # 碰撞后油门无效（车辆不能加速）
                    effective_throttle = self.throttle
                    if self.collided:
                        effective_throttle = 0.0

                    # ESP 扭矩削减（作用于驾驶员油门）
                    effective_throttle = effective_throttle * (1.0 - esp_output['torque_reduction'])

                    # ESP 差动制动：目标轮压力(bar) → 附加制动扭矩(N·m)
                    esp_brake_torques = {
                        w: p * self.ESP_BAR_TO_TORQUE
                        for w, p in esp_output['brake_pressures'].items()
                    }

                    # 车轮级底盘系统（ABS/EBD/ASR/扭矩矢量 + 车轮动力学）
                    self.chassis.sync_wheel_speeds(self.vehicle)
                    chassis_out = self.chassis.compute(
                        vehicle_state=state,
                        steering_deg=self.steering,
                        throttle=effective_throttle,
                        brake=self.brake,
                        mu=self.mu,
                        dt=dt,
                        esp_brake_torques=esp_brake_torques,
                    )
                    self.last_chassis_out = chassis_out

                    # 车辆模型步进（轮级纵向力 + 抓地缩放 + ESP/扭矩矢量力矩）
                    self.vehicle.step(
                        steering_angle_deg=self.steering,
                        throttle=effective_throttle,
                        brake=self.brake,
                        Mz_esp=esp_output['Mz_applied'],
                        mu=self.mu,
                        dt=dt,
                        wheel_forces=chassis_out['wheel_forces'],
                        grip_scales=chassis_out['grip_scales'],
                        Mz_tv=chassis_out['Mz_tv'],
                    )

                    self.sim_time += dt
                except Exception as e:
                    # 单步仿真异常不能杀死整个仿真线程
                    print(f"[仿真错误] {e}")

            # 100Hz → sleep 10ms
            time.sleep(0.01)

    def push_data(self):
        """
        后台数据推送循环 — 20Hz 推送频率

        将仿真数据打包为 JSON 通过 WebSocket 推送给前端
        """
        while self.running:
            if not self.paused:
                try:
                    data = self._build_push_data()
                    # 清洗 numpy 类型，确保 JSON 可序列化
                    data = convert_numpy_types(data)
                    socketio.emit('sim_data', data)
                except Exception as e:
                    # 单次推送失败（如序列化异常）不能杀死推送线程
                    print(f"[推送错误] {e}")
            # 20Hz → sleep 50ms
            time.sleep(0.05)

    def _build_push_data(self) -> dict:
        """
        构建推送给前端的完整数据包

        返回:
            dict: 包含车辆状态、轮速、传感器、ESP状态、输入和场景信息
        """
        state = self.vehicle.get_state()
        wheel_speeds = self.vehicle.get_wheel_speeds()  # rad/s
        alpha_f, alpha_r = self.vehicle.get_slip_angles()  # rad
        esp_out = self.last_esp_output or self.esp.compute(
            state, self.steering, self.mu, self.esp_enabled
        )
        chassis_out = self.last_chassis_out
        chassis_telem = chassis_out["telemetry"] if chassis_out else None

        # 轮胎半径用于轮速转换
        r = self.vehicle.wheel_radius

        # 各轮线速度 [km/h]：有车轮动力学时用真实轮速（能体现抱死=0/空转超速）
        if chassis_telem:
            wheel_speed_kmh = {w: chassis_telem["omega"][w] * r * 3.6 for w in ("FL", "FR", "RL", "RR")}
        else:
            wheel_speed_kmh = {k: v * r * 3.6 for k, v in wheel_speeds.items()}

        # 各轮滑移率：有车轮动力学时用真实值，否则用侧偏角估算
        if chassis_telem:
            slip_map = {w: chassis_telem["slip"][w] for w in ("FL", "FR", "RL", "RR")}
        else:
            slip_f = abs(np.degrees(alpha_f)) / 10.0  # 归一化到大约 0~1 范围
            slip_r = abs(np.degrees(alpha_r)) / 10.0
            slip_map = {"FL": slip_f, "FR": slip_f, "RL": slip_r, "RR": slip_r}

        # 各轮制动压力
        pressures = esp_out.get('brake_pressures', {})

        # 当前时代信息
        era = self.chassis.era

        # 构建推送数据
        data = {
            "time": round(self.sim_time, 3),
            "vehicle": {
                "x": round(state["x"], 3),
                "y": round(state["y"], 3),
                "psi": round(state["psi"], 4),
                "vx": round(state["v_x"], 3),
                "vy": round(state["v_y"], 3),
                "omega_z": round(np.degrees(state["omega_z"]), 2),
                "beta": round(state["beta_deg"], 2),
                "speed_kmh": round(state["speed_kmh"], 1),
            },
            "wheels": {
                "fl": {
                    "speed": round(wheel_speed_kmh["FL"], 1),
                    "slip": round(min(slip_map["FL"], 1.0), 3),
                    "pressure": round(pressures.get("FL", 0.0), 1),
                    "locked": bool(chassis_telem["locked"]["FL"]) if chassis_telem else False,
                },
                "fr": {
                    "speed": round(wheel_speed_kmh["FR"], 1),
                    "slip": round(min(slip_map["FR"], 1.0), 3),
                    "pressure": round(pressures.get("FR", 0.0), 1),
                    "locked": bool(chassis_telem["locked"]["FR"]) if chassis_telem else False,
                },
                "rl": {
                    "speed": round(wheel_speed_kmh["RL"], 1),
                    "slip": round(min(slip_map["RL"], 1.0), 3),
                    "pressure": round(pressures.get("RL", 0.0), 1),
                    "locked": bool(chassis_telem["locked"]["RL"]) if chassis_telem else False,
                },
                "rr": {
                    "speed": round(wheel_speed_kmh["RR"], 1),
                    "slip": round(min(slip_map["RR"], 1.0), 3),
                    "pressure": round(pressures.get("RR", 0.0), 1),
                    "locked": bool(chassis_telem["locked"]["RR"]) if chassis_telem else False,
                },
            },
            "sensors": {
                "steering_angle": round(self.steering, 1),
                "yaw_rate": round(np.degrees(state["omega_z"]), 2),
                "lateral_accel": round(state["ay"], 2),
                "lateral_accel_g": round(state["ay_g"], 3),
                "brake_pressure": round(max(pressures.values()) if pressures else 0.0, 1),
            },
            "esp": {
                "active": esp_out["active"],
                "enabled": esp_out["enabled"],
                "intensity": round(esp_out["intensity"], 3),
                "mz_applied": round(esp_out["Mz_applied"], 1),
                "target_wheel": esp_out["target_wheel"],
                "omega_desired": round(esp_out["omega_desired"], 2),
                "omega_error": round(esp_out["omega_error"], 2),
                "torque_reduction": round(esp_out["torque_reduction"], 3),
            },
            "inputs": {
                "steering": round(self.steering, 1),
                "throttle": round(self.throttle, 3),
                "brake": round(self.brake, 3),
            },
            "scenario": {
                # 字段保留兼容前端，无布局方案
                "name": None,
                "playing": False,
                "mu": self.mu,
            },
            # 当前技术世代与底盘系统状态
            "era": {
                "id": era["id"],
                "name": era["name"],
                "year": era["year"],
                "desc": era["desc"],
                "systems": dict(self.chassis.systems),
            },
            "systems": {
                "abs_active": bool(chassis_telem["abs_active"]) if chassis_telem else False,
                "asr_active": bool(chassis_telem["asr_active"]) if chassis_telem else False,
                "asr_throttle": float(chassis_telem["asr_throttle"]) if chassis_telem else 0.0,
                "tv_moment": float(chassis_telem["tv_moment"]) if chassis_telem else 0.0,
                "grip": dict(chassis_telem["grip"]) if chassis_telem else {"front": 1.0, "rear": 1.0},
            },
            "sim": {
                "paused": self.paused,
                "slowmo_rate": self.slowmo_rate,
            },
            # 障碍物列表（世界坐标）
            "obstacles": [
                {"id": o["id"], "x": round(o["x"], 2), "y": round(o["y"], 2), "radius": round(o["radius"], 2)}
                for o in self.obstacles
            ],
            # 碰撞状态
            "collision": {
                "hit": self.collided,
                "obstacle_id": self.collision_obstacle_id,
            },
        }
        return data


# ============ 全局引擎实例 ============
engine = SimulationEngine()


# ============ REST API 路由 ============

@app.route('/')
def index():
    """返回前端主页面"""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/control', methods=['POST'])
def api_control():
    """
    接收手动驾驶输入

    Body: {"steering": float, "throttle": float, "brake": float}
    - steering: 方向盘角度 [deg]，范围 -540~540
    - throttle: 油门 [0~1]
    - brake: 制动 [0~1]

    注：完全手动操控，无场景自动播放逻辑
    """
    try:
        data = request.get_json(force=True)

        engine.steering = float(np.clip(data.get('steering', 0), -540, 540))
        engine.throttle = float(np.clip(data.get('throttle', 0), 0, 1))
        engine.brake = float(np.clip(data.get('brake', 0), 0, 1))
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/esp_toggle', methods=['POST'])
def api_esp_toggle():
    """
    切换 ESP 开关

    Body: {"enabled": bool}
    """
    try:
        data = request.get_json(force=True)
        engine.esp_enabled = bool(data.get('enabled', True))
        return jsonify({"status": "ok", "esp_enabled": engine.esp_enabled})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/mu', methods=['POST'])
def api_mu():
    """
    设置路面附着系数

    Body: {"value": float}  — 范围 0.1 ~ 1.2
    """
    try:
        data = request.get_json(force=True)
        value = float(data.get('value', 0.9))
        engine.mu = float(np.clip(value, 0.1, 1.2))
        return jsonify({"status": "ok", "mu": engine.mu})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/eras', methods=['GET'])
def api_eras():
    """
    获取全部技术世代列表（前端时代选择器初始化用）

    Response: {"status": "ok", "eras": [...], "current": str}
    """
    try:
        return jsonify({
            "status": "ok",
            "eras": ERAS,
            "current": engine.chassis.era["id"],
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/api/era', methods=['POST'])
def api_era():
    """
    切换技术世代

    Body: {"era": "bare"|"abs"|"ebd"|"asr"|"esp"|"tv"}
    行为：配置该代系统组合 + 应用推荐路面 mu + ESP 开关同步 + 车辆回原点
    """
    try:
        data = request.get_json(force=True)
        era_id = str(data.get('era', 'esp'))
        era = engine.apply_era(era_id)
        return jsonify({
            "status": "ok",
            "era": era_id,
            "mu": engine.mu,
            "esp_enabled": engine.esp_enabled,
        })
    except KeyError:
        return jsonify({"status": "error", "message": f"unknown era: {era_id}"}), 404
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/reset', methods=['POST'])
def api_reset():
    """
    重置仿真：清空所有障碍物 + 车辆回原点 + 速度归零

    Body: {}（无需参数）
    """
    try:
        engine.reset()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/reset_position', methods=['POST'])
def api_reset_position():
    """
    回到原点：车辆传送回 (0, 0)，朝向 psi=0，速度归零，碰撞状态清除

    Body: {}（无需参数）
    注：不改变障碍物布局（被删除的障碍物不会恢复）
    """
    try:
        engine.reset_position()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/pause', methods=['POST'])
def api_pause():
    """
    暂停/恢复仿真

    Body: {"paused": bool}
    """
    try:
        data = request.get_json(force=True)
        engine.paused = bool(data.get('paused', False))
        return jsonify({"status": "ok", "paused": engine.paused})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/slowmo', methods=['POST'])
def api_slowmo():
    """
    设置慢放倍率

    Body: {"rate": float}  — 可选值 0.25, 0.5, 1.0
    """
    try:
        data = request.get_json(force=True)
        rate = float(data.get('rate', 1.0))
        # 限制在合理范围内
        engine.slowmo_rate = float(np.clip(rate, 0.1, 2.0))
        return jsonify({"status": "ok", "slowmo_rate": engine.slowmo_rate})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/obstacle/add', methods=['POST'])
def api_obstacle_add():
    """
    在指定世界坐标放置障碍物

    Body: {"x": float, "y": float}
    Response: {"status": "ok", "id": int}
    """
    try:
        data = request.get_json(force=True)
        x = float(data.get('x', 0))
        y = float(data.get('y', 0))
        obstacle_id = engine.add_obstacle(x, y)
        return jsonify({"status": "ok", "id": obstacle_id})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/obstacle/delete', methods=['POST'])
def api_obstacle_delete():
    """
    删除指定ID的障碍物

    Body: {"id": int}
    Response: {"status": "ok"} 或 {"status": "error", "message": "obstacle not found"}
    """
    try:
        data = request.get_json(force=True)
        obstacle_id = int(data.get('id', -1))
        if engine.delete_obstacle(obstacle_id):
            return jsonify({"status": "ok"})
        else:
            return jsonify({"status": "error", "message": "obstacle not found"}), 404
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/api/state', methods=['GET'])
def api_state():
    """获取当前完整仿真状态（REST 方式，用于页面刷新后恢复）"""
    try:
        data = convert_numpy_types(engine._build_push_data())
        return jsonify({"status": "ok", "data": data})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


# ============ WebSocket 事件 ============

@socketio.on('connect')
def handle_connect():
    """客户端连接时推送当前状态"""
    print("[WebSocket] 客户端已连接")
    # 立即发送一次当前状态
    try:
        data = convert_numpy_types(engine._build_push_data())
        socketio.emit('sim_data', data)
    except Exception as e:
        print(f"[推送错误] {e}")


@socketio.on('disconnect')
def handle_disconnect():
    """客户端断开连接"""
    print("[WebSocket] 客户端已断开")


@socketio.on('request_state')
def handle_request_state():
    """客户端主动请求一次状态数据"""
    try:
        data = convert_numpy_types(engine._build_push_data())
        socketio.emit('sim_data', data)
    except Exception as e:
        print(f"[推送错误] {e}")


@socketio.on('set_control')
def handle_set_control(data):
    """
    WebSocket 方式设置驾驶输入（低延迟备用通道）

    参数: {"steering": float, "throttle": float, "brake": float}
    """
    try:
        engine.steering = float(np.clip(data.get('steering', 0), -540, 540))
        engine.throttle = float(np.clip(data.get('throttle', 0), 0, 1))
        engine.brake = float(np.clip(data.get('brake', 0), 0, 1))
    except Exception:
        pass


# ============ 启动入口 ============

if __name__ == '__main__':
    # 启动仿真循环和推送循环作为后台守护线程（threading 模式）
    sim_thread = threading.Thread(target=engine.run_loop, daemon=True)
    push_thread = threading.Thread(target=engine.push_data, daemon=True)
    sim_thread.start()
    push_thread.start()

    print("=" * 50)
    print("  ESP 车身电子稳定系统 - 教学模拟程序")
    print("  访问 http://localhost:5000")
    print("  点击地图放置障碍物，点击障碍物删除")
    print("=" * 50)

    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
