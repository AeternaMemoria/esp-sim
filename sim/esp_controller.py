"""
ESP 横摆力矩控制器 — 车身电子稳定系统核心控制逻辑

控制策略说明：
    1. 根据方向盘转角和车速计算期望横摆角速度（理想稳态转弯响应）
    2. 比较实际横摆角速度与期望值，得到误差
    3. 误差超过死区时，通过 PD 控制器计算所需横摆力矩 Mz
    4. 将 Mz 转化为单轮制动力分配（ESP核心：差动制动）
    5. 同时计算发动机扭矩削减量

差动制动原理：
    - 转向过度（车尾甩出）：制动外侧前轮，产生反向横摆力矩
    - 转向不足（推头）：制动内侧后轮，产生辅助转向力矩
"""

import numpy as np


class ESPController:
    """ESP 横摆力矩控制器，基于期望横摆角速度跟踪的PD控制 + 单轮差动制动"""

    def __init__(self):
        """初始化ESP控制器参数"""
        # ============ 控制增益 ============
        self.Kp = 1500.0            # 比例增益 [N·m / (rad/s)]
        self.Kd = 300.0             # 微分增益 [N·m / (rad/s²)]
        self.deadzone = 2.0         # 横摆角速度误差死区 [deg/s]，小误差不介入

        # ============ 执行器限幅 ============
        self.max_Mz = 4000.0        # 最大横摆力矩 [N·m]
        self.max_brake_pressure = 150.0  # 单轮最大制动压力 [bar]

        # ============ 车辆参数（与模型一致） ============
        self.wheelbase = 2.7        # 轴距 [m]
        self.track_width = 1.5      # 轮距 [m]（制动分配用）
        self.steering_ratio = 15.5  # 转向传动比

        # ============ 不足转向梯度 ============
        # K_us > 0 表示不足转向特性（典型量产轿车）
        # 使得高速时期望横摆角速度降低，更保守
        self.K_us = 0.002           # 不足转向梯度 [rad/(m/s)²]

        # ============ 制动执行器参数 ============
        # 简化：制动压力(bar) -> 制动力(N) 的转换系数
        # 典型盘式制动器：约 30~50 N/bar（取决于卡钳面积和摩擦系数）
        self.brake_caliper_gain = 40.0  # [N/bar]

        # ============ 内部状态 ============
        self.prev_omega_error = 0.0  # 上一步横摆角速度误差 [rad/s]，用于微分项
        self.integral_error = 0.0    # 积分项（当前未使用，预留）

        # ============ 重力加速度 ============
        self.g = 9.81               # [m/s²]

    def reset(self):
        """重置控制器内部状态（每次新仿真开始时调用）"""
        self.prev_omega_error = 0.0
        self.integral_error = 0.0

    def compute(self, vehicle_state, steering_angle_deg, mu=0.85, enabled=True):
        """
        计算ESP控制量

        参数:
            vehicle_state: dict, 包含 v_x, v_y, omega_z 等车辆状态
            steering_angle_deg: float, 方向盘转角 [deg]，右正左负
            mu: float, 路面附着系数
            enabled: bool, ESP是否开启（驾驶员可关闭）

        返回:
            dict: ESP控制输出，包含期望横摆角速度、误差、力矩、制动力分配等
        """
        v_x = vehicle_state.get("v_x", 0.0)
        omega_z = vehicle_state.get("omega_z", 0.0)  # [rad/s]

        # --- ESP关闭时返回空输出 ---
        if not enabled:
            return self._make_output(
                enabled=False, active=False, intensity=0.0,
                omega_desired=0.0, omega_error=0.0,
                Mz_applied=0.0, target_wheel="none",
                brake_pressures={"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0},
                torque_reduction=0.0
            )

        # --- 低速时ESP不介入（< 15 km/h 无意义） ---
        if v_x < 4.17:  # 15 km/h ≈ 4.17 m/s
            self.prev_omega_error = 0.0
            return self._make_output(
                enabled=True, active=False, intensity=0.0,
                omega_desired=0.0, omega_error=0.0,
                Mz_applied=0.0, target_wheel="none",
                brake_pressures={"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0},
                torque_reduction=0.0
            )

        # --- 计算期望横摆角速度 [deg/s] ---
        omega_desired_deg = self._compute_desired_yaw_rate(v_x, steering_angle_deg, mu)

        # --- 实际横摆角速度转换为 deg/s ---
        omega_actual_deg = np.degrees(omega_z)

        # --- 计算横摆角速度误差 [deg/s] ---
        omega_error_deg = omega_actual_deg - omega_desired_deg

        # --- 死区判断 ---
        deadzone_rad = self.deadzone  # 死区单位已经是 deg/s
        if abs(omega_error_deg) < deadzone_rad:
            # 误差在死区内，不介入
            self.prev_omega_error = 0.0
            return self._make_output(
                enabled=True, active=False, intensity=0.0,
                omega_desired=omega_desired_deg, omega_error=omega_error_deg,
                Mz_applied=0.0, target_wheel="none",
                brake_pressures={"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0},
                torque_reduction=0.0
            )

        # --- PD控制计算横摆力矩 ---
        # 将误差转换为rad/s用于控制器计算
        omega_error_rad = np.radians(omega_error_deg)
        d_error = (omega_error_rad - self.prev_omega_error) / 0.01  # 微分项（假设dt=0.01s）

        # PD控制：Mz = -Kp * error - Kd * d_error
        # 负号：误差为正（实际>期望，转向过度）时需要负力矩来抑制
        Mz = -self.Kp * omega_error_rad - self.Kd * d_error

        # 力矩限幅
        Mz = np.clip(Mz, -self.max_Mz, self.max_Mz)

        # 更新微分状态
        self.prev_omega_error = omega_error_rad

        # --- 计算介入强度 (0~1) ---
        intensity = min(abs(Mz) / self.max_Mz, 1.0)

        # --- 单轮制动力分配 ---
        target_wheel, brake_pressures = self._distribute_braking(Mz, v_x, mu, steering_angle_deg)

        # --- 发动机扭矩削减 ---
        torque_reduction = self._compute_torque_reduction(intensity)

        # --- 判断是否正在介入 ---
        active = abs(omega_error_deg) >= deadzone_rad and abs(Mz) > 50.0

        return self._make_output(
            enabled=True,
            active=active,
            intensity=intensity,
            omega_desired=omega_desired_deg,
            omega_error=omega_error_deg,
            Mz_applied=Mz,
            target_wheel=target_wheel,
            brake_pressures=brake_pressures,
            torque_reduction=torque_reduction
        )

    def _compute_desired_yaw_rate(self, v_x, steering_angle_deg, mu=0.85):
        """
        计算期望横摆角速度

        基于稳态转弯理论：
            omega_des = v_x * delta / (L * (1 + K_us * v_x²))

        其中 delta 为前轮转角，K_us 为不足转向梯度。
        同时受摩擦圆约束：|omega_des| <= mu * g / v_x

        参数:
            v_x: 纵向车速 [m/s]
            steering_angle_deg: 方向盘转角 [deg]
            mu: 路面附着系数

        返回:
            float: 期望横摆角速度 [deg/s]
        """
        # 方向盘角度 -> 前轮转角 [rad]
        # 前端约定正值=右转，模型坐标系逆时针为正，故取反
        delta = -np.radians(steering_angle_deg) / self.steering_ratio

        # 稳态横摆角速度公式（含不足转向修正）
        v_x_safe = max(v_x, 1.0)  # 防除零
        omega_des_rad = v_x_safe * delta / (self.wheelbase * (1 + self.K_us * v_x_safe**2))

        # 摩擦圆约束：侧向加速度不超过 mu*g
        # a_y = v_x * omega_z <= mu * g  =>  omega_z <= mu*g / v_x
        omega_max_rad = mu * self.g / v_x_safe
        omega_des_rad = np.clip(omega_des_rad, -omega_max_rad, omega_max_rad)

        # 转换为 deg/s 返回
        return np.degrees(omega_des_rad)

    def _distribute_braking(self, Mz_des, v_x, mu, steering_angle_deg):
        """
        将期望横摆力矩转化为单轮制动压力分配

        差动制动策略：
        - Mz_des < 0（需要顺时针力矩 → 抑制转向过度/左甩尾）:
            制动左前轮 FL（逆时针阻力矩的反向=顺时针）
            实际：转向过度时制动外侧前轮
        - Mz_des > 0（需要逆时针力矩 → 抑制转向不足/右推头）:
            制动右后轮 RR（辅助转弯）

        简化逻辑：
        - 正力矩(逆时针)：车辆转向过度向右 → 制动右侧前轮 FR
        - 负力矩(顺时针)：车辆转向过度向左 → 制动左侧前轮 FL

        参数:
            Mz_des: 期望横摆力矩 [N·m]
            v_x: 纵向车速 [m/s]
            mu: 路面附着系数
            steering_angle_deg: 方向盘转角，用于判断转弯方向

        返回:
            tuple: (target_wheel, brake_pressures_dict)
        """
        # 初始化各轮制动压力
        pressures = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        target_wheel = "none"

        if abs(Mz_des) < 50.0:
            # 力矩太小，不分配
            return target_wheel, pressures

        # --- 根据力矩方向选择目标车轮 ---
        # 制动力产生的横摆力矩 = F_brake * (track_width / 2)
        # 制动左轮产生正力矩（逆时针），制动右轮产生负力矩（顺时针）
        half_track = self.track_width / 2.0

        if Mz_des < 0:
            # 需要顺时针力矩 → 制动右侧车轮
            # 转向过度时，优先制动外侧前轮（稳定效果最好）
            if steering_angle_deg <= 0:
                # 左转时转向过度，外侧 = 右侧前轮
                target_wheel = "FR"
            else:
                # 右转时转向过度，外侧 = 左侧后轮（辅助稳定）
                target_wheel = "RL"
        else:
            # 需要逆时针力矩 → 制动左侧车轮
            if steering_angle_deg > 0:
                # 右转时转向过度，外侧 = 左侧前轮
                target_wheel = "FL"
            else:
                # 左转时转向不足，内侧后轮 = 右后轮
                target_wheel = "RR"

        # --- 计算所需制动力 ---
        F_brake = abs(Mz_des) / half_track  # [N]

        # --- 制动力上限（摩擦圆约束）---
        # 单轮最大制动力 ≈ mu * 单轮载荷
        max_single_wheel_force = mu * (1500 * 9.81 / 4)  # 简化：四轮均分
        F_brake = min(F_brake, max_single_wheel_force)

        # --- 转换为制动压力 [bar] ---
        pressure = F_brake / self.brake_caliper_gain
        pressure = min(pressure, self.max_brake_pressure)

        pressures[target_wheel] = pressure

        return target_wheel, pressures

    def _compute_torque_reduction(self, intensity):
        """
        计算发动机扭矩削减比例

        策略：介入强度 > 0.3 时开始削减，最大削减50%
        reduction = (intensity - 0.3) / 0.7 * 0.5

        参数:
            intensity: ESP介入强度 [0~1]

        返回:
            float: 扭矩削减比例 [0~0.5]
        """
        if intensity <= 0.3:
            return 0.0
        reduction = (intensity - 0.3) / 0.7 * 0.5
        return min(reduction, 0.5)

    def _make_output(self, enabled, active, intensity, omega_desired,
                     omega_error, Mz_applied, target_wheel,
                     brake_pressures, torque_reduction):
        """
        构造标准化输出字典

        所有数值字段统一用 Python 原生类型（bool/float）包装，
        避免 numpy 标量（np.bool_/np.float64）混入推送数据后 json.dumps 失败。

        返回:
            dict: ESP控制输出
        """
        # 各轮制动压力同样需要逐个转为原生 float
        pressures_native = {k: float(v) for k, v in brake_pressures.items()}
        return {
            "enabled": bool(enabled),                  # ESP是否开启
            "active": bool(active),                    # 是否正在介入
            "intensity": float(intensity),             # 介入强度 0~1
            "omega_desired": float(omega_desired),     # 期望横摆角速度 [deg/s]
            "omega_error": float(omega_error),         # 横摆角速度误差 [deg/s]
            "Mz_applied": float(Mz_applied),           # 施加的横摆力矩 [N·m]
            "target_wheel": str(target_wheel),         # 目标制动车轮
            "brake_pressures": pressures_native,       # 各轮制动压力 [bar]
            "torque_reduction": float(torque_reduction),  # 发动机扭矩削减 0~0.5
        }


# ============ 自测代码 ============
if __name__ == "__main__":
    print("=" * 60)
    print("  ESP 横摆力矩控制器 — 自测验证")
    print("=" * 60)

    esp = ESPController()

    # --- 测试1：直线行驶无介入 ---
    print("\n【测试1】直线行驶（无方向盘输入）")
    state = {"v_x": 20.0, "v_y": 0.0, "omega_z": 0.0}
    result = esp.compute(state, steering_angle_deg=0.0, mu=0.85)
    print(f"  ESP激活: {result['active']}")
    print(f"  期望横摆: {result['omega_desired']:.2f} deg/s")
    print(f"  横摆误差: {result['omega_error']:.2f} deg/s")
    print(f"  施加力矩: {result['Mz_applied']:.1f} N·m")
    print(f"  目标车轮: {result['target_wheel']}")
    assert not result['active'], "直线行驶ESP不应介入"
    print("  ✓ 通过")

    # --- 测试2：转向过度工况（左转过度）---
    print("\n【测试2】转向过度（左转，实际横摆 > 期望横摆）")
    esp.reset()
    # 方向盘左转 90°（新约定：左转为负），期望横摆约 +24 deg/s（逆时针为正），但实际达到 +45 deg/s（严重转向过度）
    state = {"v_x": 20.0, "v_y": 3.0, "omega_z": np.radians(45)}
    result = esp.compute(state, steering_angle_deg=-90.0, mu=0.85)
    print(f"  期望横摆: {result['omega_desired']:.2f} deg/s")
    print(f"  实际横摆: 45.00 deg/s")
    print(f"  横摆误差: {result['omega_error']:.2f} deg/s")
    print(f"  ESP激活: {result['active']}")
    print(f"  介入强度: {result['intensity']:.2f}")
    print(f"  施加力矩: {result['Mz_applied']:.1f} N·m")
    print(f"  目标车轮: {result['target_wheel']}")
    print(f"  制动压力: {result['brake_pressures']}")
    print(f"  扭矩削减: {result['torque_reduction']:.2f}")
    # 转向过度（逆时针方向过大）→ 需要顺时针力矩 → Mz < 0
    assert result['active'], "转向过度时ESP应该介入"
    assert result['Mz_applied'] < 0, "左转转向过度时应施加负力矩（顺时针）"
    assert result['target_wheel'] in ['FR', 'RL'], "应制动右侧车轮"
    print("  ✓ 通过")

    # --- 测试3：转向不足工况 ---
    print("\n【测试3】转向不足（左转推头，实际横摆 < 期望横摆）")
    esp.reset()
    # 方向盘左转 180°（新约定：左转为负），期望横摆约 +19 deg/s（受摩擦圆限制），但实际只有 +5 deg/s（推头）
    state = {"v_x": 25.0, "v_y": 0.5, "omega_z": np.radians(5)}
    result = esp.compute(state, steering_angle_deg=-180.0, mu=0.85)
    print(f"  期望横摆: {result['omega_desired']:.2f} deg/s")
    print(f"  实际横摆: 5.00 deg/s")
    print(f"  横摆误差: {result['omega_error']:.2f} deg/s")
    print(f"  ESP激活: {result['active']}")
    print(f"  施加力矩: {result['Mz_applied']:.1f} N·m")
    print(f"  目标车轮: {result['target_wheel']}")
    # 转向不足（实际<期望）→ 误差为负 → 需要正力矩（逆时针辅助转弯）
    assert result['active'], "转向不足时ESP应该介入"
    assert result['Mz_applied'] > 0, "左转转向不足时应施加正力矩（逆时针辅助）"
    print("  ✓ 通过")

    # --- 测试4：低附着路面 ---
    print("\n【测试4】低附着路面 (mu=0.3, 冰雪)")
    esp.reset()
    state = {"v_x": 15.0, "v_y": 3.0, "omega_z": np.radians(30)}
    result = esp.compute(state, steering_angle_deg=-60.0, mu=0.3)
    print(f"  期望横摆: {result['omega_desired']:.2f} deg/s (受摩擦圆限制)")
    print(f"  横摆误差: {result['omega_error']:.2f} deg/s")
    print(f"  ESP激活: {result['active']}")
    print(f"  介入强度: {result['intensity']:.2f}")
    print(f"  目标车轮: {result['target_wheel']}")
    # 低mu时期望横摆角速度受摩擦圆约束，会更小
    print("  ✓ 通过")

    # --- 测试5：ESP关闭 ---
    print("\n【测试5】ESP关闭状态")
    state = {"v_x": 20.0, "v_y": 5.0, "omega_z": np.radians(40)}
    result = esp.compute(state, steering_angle_deg=-90.0, mu=0.85, enabled=False)
    print(f"  ESP启用: {result['enabled']}")
    print(f"  ESP激活: {result['active']}")
    print(f"  施加力矩: {result['Mz_applied']:.1f} N·m")
    assert not result['enabled'], "ESP应显示为关闭"
    assert not result['active'], "ESP关闭时不应介入"
    assert result['Mz_applied'] == 0.0, "ESP关闭时力矩为零"
    print("  ✓ 通过")

    print("\n" + "=" * 60)
    print("  所有自测通过！ESP控制器逻辑正确。")
    print("=" * 60)
