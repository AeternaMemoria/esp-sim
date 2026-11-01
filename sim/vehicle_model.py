"""
二自由度自行车模型 — ESP 教学仿真核心动力学模块

物理说明：
    将四轮车辆简化为前后各一个等效轮胎的"自行车"模型，
    考虑纵向(v_x)、侧向(v_y)运动和横摆(omega_z)三个自由度。
    轮胎侧偏力采用线性模型加摩擦圆限幅，适用于中小侧偏角工况。

坐标系定义（右手系）：
    - x轴：车辆前进方向（纵向）
    - y轴：车辆左侧方向（侧向）
    - z轴：垂直向上
    - psi：航向角，逆时针为正
    - omega_z：横摆角速度，逆时针为正（车辆左转时为正）
"""

import numpy as np


class VehicleModel:
    """二自由度自行车模型，包含纵向/侧向/横摆动力学"""

    def __init__(self):
        """初始化车辆参数和状态"""
        # ============ 车辆物理参数（典型轿车） ============
        self.mass = 1500.0          # 整车质量 [kg]
        self.wheelbase = 2.7        # 轴距 L = lf + lr [m]
        self.lf = 1.35              # 质心到前轴距离 [m]
        self.lr = 1.35              # 质心到后轴距离 [m]
        self.track_width = 1.55     # 轮距（左右轮中心距）[m]
        self.wheel_radius = 0.3     # 轮胎滚动半径 [m]
        self.Iz = 2500.0            # 绕z轴横摆转动惯量 [kg·m²]
        self.steering_ratio = 15.5  # 转向传动比（方向盘角度/前轮转角）

        # ============ 轮胎参数 ============
        self.Cf = 80000.0           # 前轮侧偏刚度 [N/rad]
        self.Cr = 80000.0           # 后轮侧偏刚度 [N/rad]

        # ============ 纵向力参数 ============
        self.max_drive_force = 3000.0    # 最大驱动力 [N]
        self.max_brake_force = 6000.0    # 最大制动力 [N]

        # ============ 静态载荷分配 ============
        self.g = 9.81               # 重力加速度 [m/s²]
        self.Fz_front = self.mass * self.g * self.lr / self.wheelbase  # 前轴静态载荷 [N]
        self.Fz_rear = self.mass * self.g * self.lf / self.wheelbase   # 后轴静态载荷 [N]

        # ============ 保护限幅参数 ============
        self.min_vx = 0.5           # 最小纵向速度，避免除零 [m/s]
        self.max_omega_z = np.radians(200)  # 横摆角速度限幅 ±200 deg/s [rad/s]

        # ============ 初始化状态 ============
        self.reset()

    def reset(self, v_x=0.0, x=0.0, y=0.0, psi=0.0):
        """
        重置车辆状态到初始条件

        参数:
            v_x: 初始纵向速度 [m/s]
            x:   初始全局X坐标 [m]
            y:   初始全局Y坐标 [m]
            psi: 初始航向角 [rad]，0为正东方向
        """
        self.v_x = float(v_x)       # 纵向车速 [m/s]
        self.v_y = 0.0              # 侧向车速 [m/s]
        self.omega_z = 0.0          # 横摆角速度 [rad/s]
        self.x = float(x)           # 全局X坐标 [m]
        self.y = float(y)           # 全局Y坐标 [m]
        self.psi = float(psi)       # 航向角 [rad]

        # 仿真时间
        self.time = 0.0             # 累计仿真时间 [s]

    def step(self, steering_angle_deg, throttle, brake, Mz_esp=0.0, mu=0.85, dt=0.01,
             wheel_forces=None, grip_scales=None, Mz_tv=0.0):
        """
        执行一步仿真（前向欧拉积分）

        参数:
            steering_angle_deg: 方向盘转角 [deg]，范围 -540~540，右正左负
            throttle:           油门开度 [0~1]
            brake:              制动踏板力度 [0~1]
            Mz_esp:             ESP施加的横摆力矩 [N·m]，逆时针为正
            mu:                 路面附着系数（干沥青~0.85，湿滑~0.4，冰雪~0.15）
            dt:                 仿真时间步长 [s]
            wheel_forces:       可选，各轮纵向力 {"FL": Fx, ...} [N]，正为驱动。
                                由 chassis_systems 提供时，取代简化油门/制动模型，
                                能体现抱死/滑移等轮级现象
            grip_scales:        可选，前后轴侧向抓地余量 {"front": 0~1, "rear": 0~1}，
                                摩擦圆耦合：纵向滑移越大，侧向力上限越低
            Mz_tv:              扭矩矢量横摆力矩 [N·m]，逆时针为正

        返回:
            state: dict 当前完整车辆状态
        """
        # --- 输入限幅 ---
        steering_angle_deg = np.clip(steering_angle_deg, -540.0, 540.0)
        throttle = np.clip(throttle, 0.0, 1.0)
        brake = np.clip(brake, 0.0, 1.0)

        # --- 方向盘角度转换为前轮转角 [rad] ---
        # 前端约定：正值=右转；模型坐标系：逆时针为正（左转）
        # 因此取反：正方向盘角 → 负 delta → 右转
        delta = -np.radians(steering_angle_deg) / self.steering_ratio

        # --- 确保纵向速度不为零（保护性下限） ---
        v_x_eff = max(abs(self.v_x), self.min_vx)
        # 保留方向符号
        v_x_signed = v_x_eff if self.v_x >= 0 else -v_x_eff

        # --- 计算轮胎侧偏角 ---
        # 前轮侧偏角 = 前轮转角 - 前轮速度方向角
        alpha_f = delta - np.arctan2(self.v_y + self.lf * self.omega_z, v_x_signed)
        # 后轮侧偏角 = - 后轮速度方向角（后轮无转角）
        alpha_r = -np.arctan2(self.v_y - self.lr * self.omega_z, v_x_signed)

        # --- 轮胎侧偏力（线性模型 + 摩擦圆限幅）---
        # 侧偏力 = 侧偏刚度 × 侧偏角（正侧偏角产生正侧向力）
        # 注：alpha_f = delta - velocity_angle，正值表示轮胎指向外侧
        #     产生的力指向内侧（恢复力），符号由公式自然保证
        Fyf_raw = self.Cf * alpha_f
        Fyr_raw = self.Cr * alpha_r

        # 摩擦圆限幅：侧向力不超过 mu * Fz
        # grip_scales 提供时叠加抱死/滑移的抓地损失（裸车抱死后转向失效的关键）
        grip_front = grip_scales["front"] if grip_scales else 1.0
        grip_rear = grip_scales["rear"] if grip_scales else 1.0
        Fyf_max = mu * self.Fz_front * grip_front
        Fyr_max = mu * self.Fz_rear * grip_rear
        Fyf = np.clip(Fyf_raw, -Fyf_max, Fyf_max)
        Fyr = np.clip(Fyr_raw, -Fyr_max, Fyr_max)

        # --- 纵向力 ---
        if wheel_forces is not None:
            # 轮级模式：由 chassis_systems 提供各轮纵向力（含抱死/滑移/ABS 调压效果）
            Fx_total = sum(wheel_forces.values())
        else:
            # 简化模式（向后兼容）：油门/制动直接映射为合力
            # 驱动力（分配到后轴，简化为前驱/后驱均可）
            Fx_drive = throttle * self.max_drive_force
            # 制动力（与速度方向相反）
            Fx_brake = -brake * self.max_brake_force * np.sign(v_x_signed)
            # 纵向合力
            Fx_total = Fx_drive + Fx_brake

        # --- 二自由度运动学方程 ---
        # 纵向加速度
        dv_x = (Fx_total + Fyf * np.sin(delta) + self.mass * self.v_y * self.omega_z) / self.mass
        # 侧向加速度
        dv_y = (Fyf * np.cos(delta) + Fyr - self.mass * self.v_x * self.omega_z) / self.mass
        # 横摆角加速度（Mz_esp：ESP 差动制动；Mz_tv：扭矩矢量，均逆时针为正）
        domega_z = (self.lf * Fyf * np.cos(delta) - self.lr * Fyr + Mz_esp + Mz_tv) / self.Iz

        # --- 全局坐标运动学 ---
        dx = self.v_x * np.cos(self.psi) - self.v_y * np.sin(self.psi)
        dy = self.v_x * np.sin(self.psi) + self.v_y * np.cos(self.psi)
        dpsi = self.omega_z

        # --- 前向欧拉积分 ---
        self.v_x += dv_x * dt
        self.v_y += dv_y * dt
        self.omega_z += domega_z * dt
        self.x += dx * dt
        self.y += dy * dt
        self.psi += dpsi * dt

        # --- 保护限幅 ---
        # 纵向速度不允许为负（不倒车）
        self.v_x = max(self.v_x, 0.0)
        # 横摆角速度限幅
        self.omega_z = np.clip(self.omega_z, -self.max_omega_z, self.max_omega_z)
        # 航向角归一化到 [-pi, pi]
        self.psi = (self.psi + np.pi) % (2 * np.pi) - np.pi

        # --- 更新仿真时间 ---
        self.time += dt

        return self.get_state()

    def get_state(self):
        """
        获取当前完整车辆状态

        返回:
            dict: 包含所有状态变量和衍生量
        """
        return {
            "v_x": self.v_x,                       # 纵向车速 [m/s]
            "v_y": self.v_y,                       # 侧向车速 [m/s]
            "omega_z": self.omega_z,               # 横摆角速度 [rad/s]
            "omega_z_deg": np.degrees(self.omega_z),  # 横摆角速度 [deg/s]
            "x": self.x,                           # 全局X坐标 [m]
            "y": self.y,                           # 全局Y坐标 [m]
            "psi": self.psi,                       # 航向角 [rad]
            "psi_deg": np.degrees(self.psi),       # 航向角 [deg]
            "speed_kmh": self.v_x * 3.6,           # 车速 [km/h]
            "beta_deg": np.degrees(self.get_beta()),  # 质心侧偏角 [deg]
            "ay": self.get_lateral_accel(),        # 侧向加速度 [m/s²]
            "ay_g": self.get_lateral_accel() / self.g,  # 侧向加速度 [g]
            "time": self.time,                     # 仿真时间 [s]
        }

    def get_wheel_speeds(self):
        """
        计算四轮轮速，考虑横摆运动导致的内外轮差

        横摆时外侧车轮线速度 = v_x + omega_z * (track_width/2)
        内侧车轮线速度 = v_x - omega_z * (track_width/2)

        返回:
            dict: 各轮角速度 [rad/s]
                  {"FL": ..., "FR": ..., "RL": ..., "RR": ...}
        """
        half_track = self.track_width / 2.0
        v_x_eff = max(self.v_x, self.min_vx)

        # 前轮考虑侧向速度分量（前轮处侧向速度 = v_y + lf * omega_z）
        v_lat_front = self.v_y + self.lf * self.omega_z
        # 后轮处侧向速度 = v_y - lr * omega_z
        v_lat_rear = self.v_y - self.lr * self.omega_z

        # 各轮纵向线速度（含横摆内外轮差）
        # 左轮（外侧取决于转向方向）
        v_FL = np.sqrt((v_x_eff + self.omega_z * half_track)**2 + v_lat_front**2)
        v_FR = np.sqrt((v_x_eff - self.omega_z * half_track)**2 + v_lat_front**2)
        v_RL = np.sqrt((v_x_eff + self.omega_z * half_track)**2 + v_lat_rear**2)
        v_RR = np.sqrt((v_x_eff - self.omega_z * half_track)**2 + v_lat_rear**2)

        # 转换为角速度 = 线速度 / 轮胎半径
        return {
            "FL": v_FL / self.wheel_radius,
            "FR": v_FR / self.wheel_radius,
            "RL": v_RL / self.wheel_radius,
            "RR": v_RR / self.wheel_radius,
        }

    def get_slip_angles(self):
        """
        计算前后轮侧偏角

        侧偏角定义：轮胎指向方向与实际速度方向之间的夹角
        正值表示轮胎指向外侧（相对于速度方向）

        返回:
            tuple: (alpha_f, alpha_r) 前后轮侧偏角 [rad]
        """
        v_x_eff = max(abs(self.v_x), self.min_vx)
        v_x_signed = v_x_eff if self.v_x >= 0 else -v_x_eff

        # 简化：此处使用零方向盘转角的侧偏角（控制器内部会重新算含转角的）
        alpha_f = np.arctan2(self.v_y + self.lf * self.omega_z, v_x_signed)
        alpha_r = np.arctan2(self.v_y - self.lr * self.omega_z, v_x_signed)

        return alpha_f, alpha_r

    def get_lateral_accel(self):
        """
        计算车辆质心侧向加速度 [m/s²]

        a_y = dv_y/dt + v_x * omega_z（含向心加速度分量）
        简化近似：a_y ≈ v_x * omega_z（稳态时主导项）
        """
        return self.v_x * self.omega_z

    def get_beta(self):
        """
        计算质心侧偏角 beta [rad]

        beta = atan(v_y / v_x)
        表示车辆实际运动方向与车头指向之间的夹角
        """
        v_x_eff = max(abs(self.v_x), self.min_vx)
        return np.arctan2(self.v_y, v_x_eff)

    def force_stop(self):
        """
        碰撞后强制停车

        将纵向速度、侧向速度和横摆角速度全部置零，
        位置(x, y)和航向角(psi)保持不变（车停在碰撞位置）。
        """
        self.v_x = 0.0
        self.v_y = 0.0
        self.omega_z = 0.0

    def get_reference_speed(self):
        """
        获取参考车速 [km/h]

        参考车速用于ESP判断，取纵向车速（简化处理，
        实际系统会通过四轮轮速融合估计）
        """
        return self.v_x * 3.6


# ============ 自测代码 ============
if __name__ == "__main__":
    print("=" * 60)
    print("  二自由度自行车模型 — 自测验证")
    print("=" * 60)

    vehicle = VehicleModel()

    # --- 测试1：直线加速 ---
    print("\n【测试1】直线加速 (油门50%, 5秒)")
    vehicle.reset(v_x=0)
    dt = 0.01
    for i in range(500):  # 5秒
        state = vehicle.step(
            steering_angle_deg=0,
            throttle=0.5,
            brake=0.0,
            Mz_esp=0.0,
            mu=0.85,
            dt=dt
        )
    print(f"  终态车速: {state['speed_kmh']:.1f} km/h")
    print(f"  行驶距离: {state['x']:.1f} m")
    print(f"  横摆角速度: {state['omega_z_deg']:.2f} deg/s (应接近0)")
    print(f"  质心侧偏角: {state['beta_deg']:.2f} deg (应接近0)")
    assert state['speed_kmh'] > 10, "直线加速应该达到一定速度"
    assert abs(state['omega_z_deg']) < 1.0, "直线行驶横摆角速度应接近零"
    print("  ✓ 通过")

    # --- 测试2：稳态转弯 ---
    print("\n【测试2】稳态转弯 (方向盘180°, 60km/h初速, 5秒)")
    vehicle.reset(v_x=60/3.6)
    for i in range(500):  # 5秒
        state = vehicle.step(
            steering_angle_deg=180,
            throttle=0.0,
            brake=0.0,
            Mz_esp=0.0,
            mu=0.85,
            dt=dt
        )
    print(f"  终态车速: {state['speed_kmh']:.1f} km/h")
    print(f"  横摆角速度: {state['omega_z_deg']:.2f} deg/s")
    print(f"  航向角变化: {state['psi_deg']:.1f} deg")
    print(f"  质心侧偏角: {state['beta_deg']:.2f} deg")
    print(f"  侧向加速度: {state['ay_g']:.3f} g")
    assert abs(state['omega_z_deg']) > 1.0, "转弯应有横摆角速度"
    print("  ✓ 通过")

    # --- 测试3：制动 ---
    print("\n【测试3】紧急制动 (100km/h, 全力制动)")
    vehicle.reset(v_x=100/3.6)
    state = vehicle.get_state()  # 获取重置后的状态
    steps = 0
    while state['speed_kmh'] > 1.0 and steps < 1000:
        state = vehicle.step(
            steering_angle_deg=0,
            throttle=0.0,
            brake=1.0,
            Mz_esp=0.0,
            mu=0.85,
            dt=dt
        )
        steps += 1
    braking_distance = state['x']
    braking_time = steps * dt
    print(f"  制动距离: {braking_distance:.1f} m")
    print(f"  制动时间: {braking_time:.2f} s")
    print(f"  终态车速: {state['speed_kmh']:.1f} km/h")
    # 模型中 max_brake_force=6000N，1500kg车产生 4m/s² 减速度
    # 100km/h 制动距离理论值 = v²/(2a) = 27.78²/(2*4) ≈ 96m
    assert 60 < braking_distance < 120, f"制动距离 {braking_distance:.1f}m 不在合理范围"
    print("  ✓ 通过")

    # --- 测试4：轮速计算 ---
    print("\n【测试4】轮速计算验证")
    vehicle.reset(v_x=20.0)  # 72 km/h
    vehicle.omega_z = np.radians(10)  # 10 deg/s 左转
    wheel_speeds = vehicle.get_wheel_speeds()
    print(f"  FL (左前/外侧): {wheel_speeds['FL']:.1f} rad/s")
    print(f"  FR (右前/内侧): {wheel_speeds['FR']:.1f} rad/s")
    print(f"  RL (左后/外侧): {wheel_speeds['RL']:.1f} rad/s")
    print(f"  RR (右后/内侧): {wheel_speeds['RR']:.1f} rad/s")
    assert wheel_speeds['FL'] > wheel_speeds['FR'], "左转时左轮（外侧）应更快"
    print("  ✓ 通过")

    print("\n" + "=" * 60)
    print("  所有自测通过！模型行为合理。")
    print("=" * 60)
