"""
chassis_systems.py — 底盘电子系统世代模拟（教学用）

为 ESP 教学仿真提供"技术演进史"各代系统的模拟：
    第 1 代  bare  裸车（1978 前）  ：无任何电子系统，猛刹抱死、冰面空转
    第 2 代  abs   ABS（1978）      ：滑移率闭环调压，防抱死保住转向力
    第 3 代  ebd   +EBD（80 年代末）：制动时降低后轴压力比例，防后轮先锁甩尾
    第 4 代  asr   +ASR（1986）     ：驱动轮滑移超限自动削减油门
    第 5 代  esp   +ESP（1995）     ：横摆闭环差动制动（复用 esp_controller.py）
    第 6 代  tv    扭矩矢量（电驱） ：按转向分配左右驱动力矩，用"推"辅助横摆

物理基础（车轮旋转动力学，简化单轮模型）：
    J_w * dω/dt = T_drive - T_brake - F_x * r
    滑移率 κ：制动 (v - ωr)/v，驱动 (ωr - v)/ωr
    μ-κ 曲线：线性上升到峰值（κ≈0.12）后衰减到滑动摩擦（≈0.82×峰值）
    摩擦圆耦合：纵向滑移越大，侧向抓地余量越小（grip_scale）

坐标系与符号约定（与 vehicle_model.py / esp_controller.py 一致）：
    - 方向盘 steering：右正左负
    - 车辆模型：x 前、y 左、psi/omega_z 逆时针为正
    - 因此扭矩矢量：steering>0（右转）→ Mz_tv 为负（顺时针）
"""

import numpy as np


# ============ 时代注册表 ============
# systems 中 esp 标志决定 esp_controller 是否启用；
# suggest_mu 为教学推荐路面（加载时代时自动切换，前端同步路面按钮）
ERAS = [
    {
        "id": "bare",
        "name": "第 1 代 · 裸车（1978 前）",
        "year": "1978 前",
        "desc": "没有任何电子系统。猛踩刹车 → 车轮抱死 → 方向盘失效；冰面给油 → 驱动轮空转。这是所有故事的起点。",
        "systems": {"abs": False, "ebd": False, "asr": False, "esp": False, "tv": False},
        "suggest_mu": 0.5,
    },
    {
        "id": "abs",
        "name": "第 2 代 · ABS（1978）",
        "year": "1978",
        "desc": "防抱死制动：轮速传感器 + 液压调压，把滑移率控制在峰值附附近。刹得住，还转得动。",
        "systems": {"abs": True, "ebd": False, "asr": False, "esp": False, "tv": False},
        "suggest_mu": 0.5,
    },
    {
        "id": "ebd",
        "name": "第 3 代 · +EBD（80 年代末）",
        "year": "1980s",
        "desc": "电子制动力分配：制动载荷前移时降低后轴压力比例，防止后轮先锁导致甩尾。",
        "systems": {"abs": True, "ebd": True, "asr": False, "esp": False, "tv": False},
        "suggest_mu": 0.5,
    },
    {
        "id": "asr",
        "name": "第 4 代 · +ASR（1986）",
        "year": "1986",
        "desc": "驱动防滑：驱动轮滑移率超过 30% 自动削减油门。「砍扭矩」手法后来被 ESP 继承。",
        "systems": {"abs": True, "ebd": True, "asr": True, "esp": False, "tv": False},
        "suggest_mu": 0.2,
    },
    {
        "id": "esp",
        "name": "第 5 代 · +ESP（1995）",
        "year": "1995",
        "desc": "电子稳定程序：比较「意图 vs 现实」，差动制动 + 扭矩削减修正横摆。教学主线。",
        "systems": {"abs": True, "ebd": True, "asr": True, "esp": True, "tv": False},
        "suggest_mu": 0.9,
    },
    {
        "id": "tv",
        "name": "第 6 代 · 扭矩矢量（电驱时代）",
        "year": "2010s",
        "desc": "扭矩矢量分配：按转向给外侧轮多分驱动扭矩，用「推」代替「拖」产生横摆力矩，不浪费能量。",
        "systems": {"abs": True, "ebd": True, "asr": True, "esp": True, "tv": True},
        "suggest_mu": 0.9,
    },
]


def get_era(era_id):
    """按 id 获取时代定义，未找到返回 None"""
    for era in ERAS:
        if era["id"] == era_id:
            return era
    return None


class ChassisSystems:
    """
    车轮级动力学 + 各代底盘电子系统

    用法（仿真主循环每步）：
        chassis.sync_wheel_speeds(vehicle)               # 车身状态 → 车轮状态
        out = chassis.compute(state, steering, throttle, brake, Mz_esp, mu, dt)
        vehicle.step(..., wheel_forces=out["wheel_forces"],
                     grip_scales=out["grip_scales"], Mz_tv=out["Mz_tv"])
    """

    WHEELS = ("FL", "FR", "RL", "RR")

    def __init__(self):
        # ============ 车轮物理参数 ============
        self.r = 0.3                 # 轮胎滚动半径 [m]
        self.J = 2.5                 # 单轮等效转动惯量 [kg·m²]（含制动盘/半轴）
        self.max_brake_torque = 2100.0   # 单轮满制动扭矩 [N·m]（对应约 7000N 制动力）
        self.max_drive_torque = 1000.0   # 单后轮满驱动扭矩 [N·m]（后驱，总驱动力约 6600N）

        # ============ μ-κ 曲线参数 ============
        self.kappa_peak = 0.12       # 峰值附着滑移率
        self.mu_slide_ratio = 0.82   # 滑动摩擦 / 峰值附着 比例（抱死后力下降）

        # ============ ABS 参数 ============
        # 附着前馈 + 滑移闭环：T_hold = μ·Fz·r 估计峰值附着扭矩，
        # 滑移超过目标快速泄压，正常时向前馈值收敛（收敛快、稳态准）
        self.abs_slip_target = 0.12  # 目标滑移率（峰值附着点）
        self.abs_release_rate = 30000.0  # 最大泄压速率 [N·m/s]
        self.abs_apply_rate = 20000.0    # 最大建压速率 [N·m/s]（泄压快于建压 = 真实 ABS 特征）

        # ============ EBD 参数 ============
        self.ebd_max_rear_cut = 0.45  # 后轴压力最大削减比例

        # ============ ASR 参数 ============
        self.asr_slip_limit = 0.30    # 驱动轮滑移率目标上限（比例控制围绕此点平衡）
        self.asr_kp = 2.0             # 比例增益：滑移偏差 → 油门削减
        self.asr_kd = 0.15            # 微分增益：滑移上升趋势 → 提前削减（防超调）

        # ============ 扭矩矢量参数 ============
        self.tv_gain = 2200.0         # 方向盘角 → 横摆力矩增益 [N·m/deg]
        self.tv_max_moment = 1400.0   # 最大扭矩矢量横摆力矩 [N·m]

        # ============ 时代配置 ============
        self.era = ERAS[4]            # 默认第 5 代 ESP
        self.systems = dict(self.era["systems"])

        # ============ 车轮状态 ============
        self.wheel_omega = {w: 0.0 for w in self.WHEELS}   # 车轮角速度 [rad/s]
        self.brake_torque = {w: 0.0 for w in self.WHEELS}  # ABS 调节后的实际制动扭矩
        self.asr_throttle = 0.0        # ASR 调节后的有效油门
        self.prev_drive_slip = 0.0     # 上一步驱动轮滑移率（ASR 微分项用）
        self.last_slip = {w: 0.0 for w in self.WHEELS}
        self.last_locked = {w: False for w in self.WHEELS}
        self.last_mu_x = {w: 0.0 for w in self.WHEELS}
        self.last_force = {w: 0.0 for w in self.WHEELS}
        self.last_grip = {"front": 1.0, "rear": 1.0}
        self.abs_active = False
        self.asr_active = False
        self.tv_moment = 0.0

    # ============ 时代切换 ============

    def set_era(self, era_id):
        """切换时代，重置车轮状态。未找到的 id 抛 KeyError"""
        era = get_era(era_id)
        if era is None:
            raise KeyError(f"未知时代: {era_id}")
        self.era = era
        self.systems = dict(era["systems"])
        self.reset()
        return era

    def reset(self):
        """重置车轮状态（车辆重置/回原点时调用）"""
        for w in self.WHEELS:
            self.wheel_omega[w] = 0.0
            self.brake_torque[w] = 0.0
            self.last_slip[w] = 0.0
            self.last_locked[w] = False
            self.last_mu_x[w] = 0.0
            self.last_force[w] = 0.0
        self.last_grip = {"front": 1.0, "rear": 1.0}
        self.asr_throttle = 0.0
        self.prev_drive_slip = 0.0
        self.abs_active = False
        self.asr_active = False
        self.tv_moment = 0.0

    def sync_wheel_speeds(self, vehicle):
        """
        无输入工况下车轮跟随车速纯滚动（用于重置后/滑行时的状态同步）

        注意：有油门或有制动时绝不能同步，否则会把滑移率强制归零，
        驱动轮永远无法空转（ASR 也就无从触发）
        """
        v = max(abs(vehicle.v_x), 0.01)
        idle = all(self.brake_torque[w] <= 1.0 for w in self.WHEELS) and self.asr_throttle < 0.01
        if idle:
            for w in self.WHEELS:
                self.wheel_omega[w] = v / self.r

    # ============ 核心计算 ============

    def compute(self, vehicle_state, steering_deg, throttle, brake, mu, dt,
                esp_brake_torques=None):
        """
        执行一步车轮动力学 + 各代系统干预

        参数:
            vehicle_state: dict, 车辆状态（需含 v_x, v_y, omega_z）
            steering_deg:  方向盘角度 [deg]，右正左负
            throttle:      油门 [0~1]
            brake:         制动踏板 [0~1]
            mu:            路面附着系数
            dt:            时间步长 [s]
            esp_brake_torques: 可选，ESP 差动制动附加扭矩 {"FL": N·m, ...}，
                               叠加在驾驶员制动需求之上（同样受附着极限和 ABS 约束）

        返回:
            dict: {
                "wheel_forces": {"FL": Fx, ...},   # 各轮纵向力 [N]，正为驱动力
                "grip_scales": {"front": 0~1, "rear": 0~1},  # 轴侧向抓地余量
                "Mz_tv": float,                    # 扭矩矢量横摆力矩 [N·m]
                "telemetry": {...}                 # 仪表数据
            }
        """
        v_x = float(vehicle_state.get("v_x", 0.0))
        sys = self.systems

        # --- ASR：驱动轮滑移超限 → PD 比例削减油门 ---
        # 真实 TCS 从静止起步就工作，不设车速门槛（驱动滑移定义不依赖车速下限）
        self.asr_active = False
        effective_throttle = throttle
        drive_slip = max(self.last_slip["RL"], self.last_slip["RR"])
        if sys["asr"]:
            # 滑移变化率（预测趋势，提前干预避免峰值超调）
            slip_rate = (drive_slip - self.prev_drive_slip) / max(dt, 1e-6)
            self.prev_drive_slip = drive_slip
            if drive_slip > self.asr_slip_limit or slip_rate > 2.0:
                over = max(drive_slip - self.asr_slip_limit, 0.0) / (1.0 - self.asr_slip_limit)
                cut = self.asr_kp * over + self.asr_kd * max(slip_rate, 0.0) / 10.0
                effective_throttle = throttle * max(1.0 - cut, 0.0)
                self.asr_active = True
        else:
            self.prev_drive_slip = drive_slip
        self.asr_throttle = effective_throttle

        # --- 制动扭矩需求（驾驶员踏板 → 各轮基准扭矩）---
        # 前轴承担 55%（静态），后轴 45%
        base_torque = brake * self.max_brake_torque
        front_torque = base_torque * 0.55
        rear_torque = base_torque * 0.45

        # --- EBD：按减速度削减后轴压力（载荷前移，后轮易先锁）---
        if sys["ebd"] and brake > 0.05:
            decel = min(brake * 9.0, 9.0)                 # 估算减速度 [m/s²]
            rear_cut = min(decel / 10.0, 1.0) * self.ebd_max_rear_cut
            rear_torque *= (1.0 - rear_cut)

        # --- ABS：逐轮滑移率闭环调压 ---
        self.abs_active = False
        esp_extra = esp_brake_torques or {}
        demanded = {
            "FL": front_torque + esp_extra.get("FL", 0.0),
            "FR": front_torque + esp_extra.get("FR", 0.0),
            "RL": rear_torque + esp_extra.get("RL", 0.0),
            "RR": rear_torque + esp_extra.get("RR", 0.0),
        }
        if sys["abs"]:
            for w in self.WHEELS:
                slip = self.last_slip[w]
                # 目标滑移率下的峰值附着（线性段：μ_peak = μ 路面系数）
                Fz_w = 1500.0 * 9.81 / 4.0
                T_hold = mu * Fz_w * self.r          # 维持目标滑移所需扭矩（附着前馈估计）
                T_target = min(demanded[w], T_hold)  # 不超过驾驶员需求
                if slip > self.abs_slip_target:
                    # 滑移偏高 → 快速泄压（泄压速率快于建压 = 真实 ABS 特征）
                    self.brake_torque[w] = max(
                        T_target * 0.5,
                        self.brake_torque[w] - self.abs_release_rate * dt)
                    self.abs_active = True
                else:
                    # 滑移正常 → 向估计的保持压力快速收敛（前馈 + 比例回调）
                    gap = T_target - self.brake_torque[w]
                    step_up = min(abs(gap) * 0.3, self.abs_apply_rate * dt)
                    self.brake_torque[w] = min(demanded[w], self.brake_torque[w] + step_up)
                self.brake_torque[w] = float(np.clip(self.brake_torque[w], 0.0, demanded[w]))
        else:
            # 无 ABS：制动扭矩直接跟随踏板（会抱死）
            for w in self.WHEELS:
                self.brake_torque[w] = demanded[w]

        # --- 驱动扭矩（后驱，ASR 削减后的油门）---
        drive_torque = effective_throttle * self.max_drive_torque

        # --- 车轮动力学积分 ---
        half_track = 0.775  # 半轮距 [m]
        v_x_safe = max(abs(v_x), 0.01)
        omega_z = float(vehicle_state.get("omega_z", 0.0))

        for w in self.WHEELS:
            omega = self.wheel_omega[w]
            v_wheel = omega * self.r

            # 滑移率（制动/驱动两种定义统一处理）
            if v_wheel > v_x_safe:
                kappa = (v_wheel - v_x_safe) / v_wheel      # 驱动滑移
            else:
                kappa = (v_x_safe - v_wheel) / v_x_safe      # 制动滑移
            kappa = float(np.clip(kappa, 0.0, 1.0))

            # μ-κ 曲线：线性上升 → 峰值 → 衰减到滑动摩擦
            if kappa <= self.kappa_peak:
                mu_x = mu * kappa / self.kappa_peak
            else:
                slide = self.mu_slide_ratio + (1 - self.mu_slide_ratio) * 0.5 * \
                    (1 + np.cos(np.pi * (kappa - self.kappa_peak) / (1 - self.kappa_peak)))
                mu_x = mu * slide

            # 单轮垂直载荷（静态均分）
            Fz = 1500.0 * 9.81 / 4.0
            # 纵向力：制动为负，驱动为正（只有后轮有驱动）
            is_drive = w in ("RL", "RR")
            F_drive = drive_torque / self.r if is_drive else 0.0
            F_brake = self.brake_torque[w] / self.r
            F_available = mu_x * Fz
            # 驱动力受附着极限约束
            F_drive_eff = min(F_drive, F_available) if F_drive > 0 else 0.0
            # 制动力同样受附着极限约束
            F_brake_eff = min(F_brake, F_available)
            F_x = F_drive_eff - F_brake_eff

            # 车轮旋转动力学积分
            # J·dω/dt = T_drive - T_brake - F_x·r
            # 关键：驱动/制动扭矩都直接作用于车轮旋转（不经过地面力），
            #       否则零滑移时 μ_x=0 会把扭矩卡死，永远无法空转/抱死；
            #       传递到地面的力 F_x 受附着极限约束，反作用回车轮旋转
            T_drive = drive_torque if is_drive else 0.0   # 电机/发动机直接扭矩
            T_brake_applied = self.brake_torque[w]        # 液压直接制动扭矩
            d_omega = (T_drive - T_brake_applied - F_x * self.r) / self.J
            omega += d_omega * dt
            # 车轮不能反转；接近零速且仍在制动 → 锁死
            if omega <= 0.0 and self.brake_torque[w] > 1.0:
                omega = 0.0
            # 轮速上限保护（约 324 km/h 等效轮速，防止冰面空转数值爆炸）
            self.wheel_omega[w] = float(np.clip(omega, 0.0, 300.0))

            # 记录遥测
            self.last_slip[w] = kappa
            self.last_locked[w] = (self.wheel_omega[w] < 0.5) and (v_x_safe > 1.0)
            self.last_mu_x[w] = mu_x
            self.last_force[w] = F_x

        # --- 摩擦圆耦合：纵向滑移吃掉侧向抓地 ---
        # grip = 1 - 0.9×κ，前轴取左右轮均值（侧向力上限缩放系数）
        grip_front = 1.0 - 0.9 * 0.5 * (self.last_slip["FL"] + self.last_slip["FR"])
        grip_rear = 1.0 - 0.9 * 0.5 * (self.last_slip["RL"] + self.last_slip["RR"])
        self.last_grip = {
            "front": float(np.clip(grip_front, 0.1, 1.0)),
            "rear": float(np.clip(grip_rear, 0.1, 1.0)),
        }

        # --- 扭矩矢量：按转向分配左右驱动力矩差 ---
        # steering>0（右转）→ 需要顺时针力矩 → 模型坐标系下 Mz 为负
        self.tv_moment = 0.0
        if sys["tv"] and effective_throttle > 0.05 and v_x_safe > 3.0:
            self.tv_moment = float(-np.clip(
                steering_deg * self.tv_gain * effective_throttle,
                -self.tv_max_moment, self.tv_max_moment))

        wheel_forces = {w: self.last_force[w] for w in self.WHEELS}

        telemetry = {
            "slip": {w: round(self.last_slip[w], 3) for w in self.WHEELS},
            "locked": {w: bool(self.last_locked[w]) for w in self.WHEELS},
            "omega": {w: round(self.wheel_omega[w], 1) for w in self.WHEELS},
            "mu_x": {w: round(self.last_mu_x[w], 3) for w in self.WHEELS},
            "brake_torque": {w: round(self.brake_torque[w], 0) for w in self.WHEELS},
            "abs_active": self.abs_active,
            "asr_active": self.asr_active,
            "asr_throttle": round(self.asr_throttle, 3),
            "tv_moment": round(self.tv_moment, 1),
            "grip": {k: round(v, 2) for k, v in self.last_grip.items()},
        }

        return {
            "wheel_forces": wheel_forces,
            "grip_scales": self.last_grip,
            "Mz_tv": self.tv_moment,
            "telemetry": telemetry,
        }


# ============ 自测代码 ============
if __name__ == "__main__":
    from sim.vehicle_model import VehicleModel

    print("=" * 60)
    print("  底盘电子系统世代模拟 — 自测验证")
    print("=" * 60)

    def run_braking_test(era_id, mu, v0_kmh=100.0):
        """100% 制动直到停住，返回制动距离与是否抱死"""
        veh = VehicleModel()
        chassis = ChassisSystems()
        chassis.set_era(era_id)
        veh.reset(v_x=v0_kmh / 3.6)
        chassis.sync_wheel_speeds(veh)
        locked_seen = False
        dt = 0.01
        steps = 0
        state = veh.get_state()
        while state["v_x"] > 0.5 and steps < 3000:
            chassis.sync_wheel_speeds(veh)
            out = chassis.compute(state, 0.0, 0.0, 1.0, mu, dt)
            state = veh.step(0.0, 0.0, 0.0, Mz_esp=0.0, mu=mu, dt=dt,
                             wheel_forces=out["wheel_forces"],
                             grip_scales=out["grip_scales"],
                             Mz_tv=out["Mz_tv"])
            if any(out["telemetry"]["locked"].values()):
                locked_seen = True
            steps += 1
        return state["x"], locked_seen

    # --- 测试1：裸车 vs ABS 制动距离（干路面）---
    print("\n【测试1】100km/h 全力制动（干路面 μ=0.9）")
    d_bare, lock_bare = run_braking_test("bare", 0.9)
    d_abs, lock_abs = run_braking_test("abs", 0.9)
    print(f"  裸车：{d_bare:.1f} m，抱死={lock_bare}")
    print(f"  ABS ：{d_abs:.1f} m，抱死={lock_abs}")
    assert lock_bare, "裸车全力制动应抱死"
    assert not lock_abs, "ABS 不应抱死"
    assert d_abs < d_bare, "ABS 制动距离应更短"
    print("  ✓ 通过")

    # --- 测试2：裸车抱死后转向失效 ---
    print("\n【测试2】抱死后打方向：裸车 vs ABS（μ=0.9, 80km/h）")
    def steer_while_braking(era_id):
        veh = VehicleModel()
        chassis = ChassisSystems()
        chassis.set_era(era_id)
        veh.reset(v_x=80 / 3.6)
        chassis.sync_wheel_speeds(veh)
        state = veh.get_state()
        dt = 0.01
        for i in range(200):  # 2 秒
            chassis.sync_wheel_speeds(veh)
            out = chassis.compute(state, -180.0, 0.0, 1.0, 0.9, dt)  # 左转 180°+全力制动
            state = veh.step(-180.0, 0.0, 0.0, mu=0.9, dt=dt,
                             wheel_forces=out["wheel_forces"],
                             grip_scales=out["grip_scales"],
                             Mz_tv=out["Mz_tv"])
        return abs(state["psi_deg"]), abs(state["y"])
    psi_bare, y_bare = steer_while_braking("bare")
    psi_abs, y_abs = steer_while_braking("abs")
    print(f"  裸车：航向变化 {psi_bare:.1f}°，侧移 {y_bare:.1f} m")
    print(f"  ABS ：航向变化 {psi_abs:.1f}°，侧移 {y_abs:.1f} m")
    assert psi_abs > psi_bare * 1.5, "ABS 制动中应保留明显更强的转向能力"
    print("  ✓ 通过")

    # --- 测试3：ASR 冰面防滑 ---
    print("\n【测试3】冰面 μ=0.2 全油门起步：裸车 vs ASR")
    def launch_test(era_id):
        veh = VehicleModel()
        chassis = ChassisSystems()
        chassis.set_era(era_id)
        veh.reset(v_x=1.0)   # 高于 ASR 启用门槛 0.5 m/s
        chassis.sync_wheel_speeds(veh)
        chassis.asr_throttle = 1.0   # 防止首次 sync 把状态拉回纯滚动
        state = veh.get_state()
        dt = 0.01
        max_slip = 0.0
        for i in range(300):  # 3 秒
            chassis.sync_wheel_speeds(veh)
            out = chassis.compute(state, 0.0, 1.0, 0.0, 0.2, dt)
            state = veh.step(0.0, 1.0, 0.0, mu=0.2, dt=dt,
                             wheel_forces=out["wheel_forces"],
                             grip_scales=out["grip_scales"],
                             Mz_tv=out["Mz_tv"])
            max_slip = max(max_slip, out["telemetry"]["slip"]["RL"], out["telemetry"]["slip"]["RR"])
        return state["speed_kmh"], max_slip
    spd_bare, slip_bare = launch_test("bare")
    spd_asr, slip_asr = launch_test("asr")
    print(f"  裸车：3 秒后 {spd_bare:.1f} km/h，驱动轮最大滑移 {slip_bare:.2f}")
    print(f"  ASR ：3 秒后 {spd_asr:.1f} km/h，驱动轮最大滑移 {slip_asr:.2f}")
    assert slip_asr < slip_bare * 0.7, "ASR 应显著抑制驱动轮滑移"
    print("  ✓ 通过")

    # --- 测试3b：冰面制动 —— ABS 优势最明显的工况 ---
    print("\n【测试3b】80km/h 全力制动（冰面 μ=0.2）")
    d_bare_ice, lock_bare_ice = run_braking_test("bare", 0.2, v0_kmh=80.0)
    d_abs_ice, lock_abs_ice = run_braking_test("abs", 0.2, v0_kmh=80.0)
    print(f"  裸车：{d_bare_ice:.1f} m，抱死={lock_bare_ice}")
    print(f"  ABS ：{d_abs_ice:.1f} m，抱死={lock_abs_ice}")
    assert lock_bare_ice, "冰面裸车全力制动应抱死"
    assert not lock_abs_ice, "冰面 ABS 不应抱死"
    assert d_abs_ice < d_bare_ice * 0.9, "冰面上 ABS 制动距离应明显更短"
    print("  ✓ 通过")

    # --- 测试4：扭矩矢量符号 ---
    print("\n【测试4】扭矩矢量符号（steering>0 右转 → Mz 应为负/顺时针）")
    chassis = ChassisSystems()
    chassis.set_era("tv")
    state = {"v_x": 15.0, "v_y": 0.0, "omega_z": 0.0}
    out_r = chassis.compute(state, 90.0, 0.5, 0.0, 0.9, 0.01)
    chassis.reset()   # 清除第一次调用留下的驱动轮滑移状态，避免 ASR 干扰符号验证
    out_l = chassis.compute(state, -90.0, 0.5, 0.0, 0.9, 0.01)
    print(f"  右转 90°：Mz_tv = {out_r['Mz_tv']:.0f} N·m")
    print(f"  左转 90°：Mz_tv = {out_l['Mz_tv']:.0f} N·m")
    assert out_r["Mz_tv"] < 0, "右转时扭矩矢量力矩应为负（顺时针）"
    assert out_l["Mz_tv"] > 0, "左转时扭矩矢量力矩应为正（逆时针）"
    print("  ✓ 通过")

    # --- 测试5：时代注册表完整性 ---
    print("\n【测试5】六个时代注册表")
    for era in ERAS:
        flags = "".join(k[0].upper() if v else "-" for k, v in era["systems"].items())
        print(f"  {era['id']:>5}  [{flags}]  {era['name']}")
    assert len(ERAS) == 6
    try:
        ChassisSystems().set_era("nonexistent")
        assert False, "未知时代应抛 KeyError"
    except KeyError:
        pass
    print("  ✓ 通过")

    print("\n" + "=" * 60)
    print("  所有自测通过！各代系统行为符合预期。")
    print("=" * 60)
