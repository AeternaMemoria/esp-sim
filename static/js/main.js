/**
 * main.js — ESP 教学模拟：主控逻辑
 * 功能：WebSocket 通信、传感器面板更新、键盘快捷键、API 调用
 */

// ========== 全局应用 ==========
class ESPApp {
  constructor() {
    // 连接状态
    this.connected = false;
    this.socket = null;

    // 节流控制（最高 20Hz 发送）
    this._lastSendTime = 0;
    this._sendInterval = 50; // ms

    // 初始化各组件
    this._initComponents();
    this._initSocket();
    this._initHeaderControls();
    this._initFooter();
    this._bindControlEvents();
    this._initEra();

    console.log('[ESP] 应用初始化完成');
  }

  // ========== 组件初始化 ==========

  _initComponents() {
    // 车辆 Canvas
    const vehicleCanvas = document.getElementById('vehicle-canvas');
    this.vehicleCanvas = new VehicleCanvas(vehicleCanvas);

    // 图表
    const chartsContainer = document.getElementById('charts-container');
    this.charts = new ChartsManager(chartsContainer);

    // 驾驶控件
    const controlsContainer = document.getElementById('controls-container');
    this.controls = new Controls(controlsContainer, (inputs) => this._onInputChange(inputs));

    // 绑定控件回调
    this.controls.onPlayStateChange = (paused) => this._sendPause(paused);
    this.controls.onReset = () => this._sendReset();
    this.controls.onSpeedChange = (rate) => this._sendSlowmo(rate);

    // 传感器面板 DOM 缓存
    this.sensorEls = {
      wheelFL: document.getElementById('sensor-wheel-fl'),
      wheelFR: document.getElementById('sensor-wheel-fr'),
      wheelRL: document.getElementById('sensor-wheel-rl'),
      wheelRR: document.getElementById('sensor-wheel-rr'),
      yawRate: document.getElementById('sensor-yaw-rate'),
      latAccel: document.getElementById('sensor-lat-accel'),
      steeringAngle: document.getElementById('sensor-steering'),
      brakeFL: document.getElementById('brake-fill-fl'),
      brakeFR: document.getElementById('brake-fill-fr'),
      brakeRL: document.getElementById('brake-fill-rl'),
      brakeRR: document.getElementById('brake-fill-rr'),
      brakeValFL: document.getElementById('brake-val-fl'),
      brakeValFR: document.getElementById('brake-val-fr'),
      brakeValRL: document.getElementById('brake-val-rl'),
      brakeValRR: document.getElementById('brake-val-rr'),
      espMoment: document.getElementById('esp-moment-value'),
      espTarget: document.getElementById('esp-target-wheel'),
      // 力分配轮标记
      forceFL: document.getElementById('force-fl'),
      forceFR: document.getElementById('force-fr'),
      forceRL: document.getElementById('force-rl'),
      forceRR: document.getElementById('force-rr')
    };

    // 底部状态栏
    this.footerEls = {
      time: document.getElementById('footer-time'),
      speed: document.getElementById('footer-speed'),
      espStatus: document.getElementById('footer-esp'),
      scenario: document.getElementById('footer-scenario'),
      connection: document.getElementById('footer-connection')
    };

    // 底盘系统徽章与抱死标记
    this.sysEls = {
      badges: {
        abs: document.getElementById('sys-badge-abs'),
        ebd: document.getElementById('sys-badge-ebd'),
        asr: document.getElementById('sys-badge-asr'),
        esp: document.getElementById('sys-badge-esp'),
        tv: document.getElementById('sys-badge-tv')
      },
      statusLine: document.getElementById('sys-status-line'),
      locks: {
        fl: document.getElementById('lock-fl'),
        fr: document.getElementById('lock-fr'),
        rl: document.getElementById('lock-rl'),
        rr: document.getElementById('lock-rr')
      }
    };

    // HUD
    this.hudEls = {
      speed: document.getElementById('hud-speed'),
      scenario: document.getElementById('hud-scenario'),
      mu: document.getElementById('hud-mu'),
      collision: document.getElementById('hud-collision')
    };
  }

  // ========== WebSocket ==========

  _initSocket() {
    // 尝试连接 Socket.IO
    if (typeof io !== 'undefined') {
      try {
        this.socket = io({
          transports: ['websocket', 'polling'],
          reconnection: true,
          reconnectionDelay: 1000,
          reconnectionAttempts: Infinity
        });

        this.socket.on('connect', () => {
          this.connected = true;
          this._updateConnectionStatus(true);
          console.log('[ESP] WebSocket 已连接');
        });

        this.socket.on('disconnect', () => {
          this.connected = false;
          this._updateConnectionStatus(false);
          console.log('[ESP] WebSocket 已断开');
        });

        // 接收仿真数据
        this.socket.on('sim_data', (data) => this._onSimData(data));

        this.socket.on('connect_error', () => {
          this.connected = false;
          this._updateConnectionStatus(false);
        });
      } catch (e) {
        console.warn('[ESP] Socket.IO 连接失败，使用离线模式', e);
        this._startOfflineMode();
      }
    } else {
      console.warn('[ESP] Socket.IO 未加载，使用离线模式');
      this._startOfflineMode();
    }
  }

  /** 离线演示模式 — 未连接后端时展示静态数据 */
  _startOfflineMode() {
    this._updateConnectionStatus(false);
    // 用静态占位数据初始化界面
    const demoData = {
      time: 0,
      vehicle: { x: 0, y: 0, psi: 0, vx: 0, vy: 0, omega_z: 0, beta: 0 },
      wheels: {
        fl: { speed: 0, slip: 0, pressure: 0 },
        fr: { speed: 0, slip: 0, pressure: 0 },
        rl: { speed: 0, slip: 0, pressure: 0 },
        rr: { speed: 0, slip: 0, pressure: 0 }
      },
      sensors: { steering_angle: 0, yaw_rate: 0, lateral_accel: 0, brake_pressure: 0 },
      esp: { active: false, enabled: true, intensity: 0, mz_applied: 0, target_wheel: null, omega_desired: 0 },
      inputs: { steering: 0, throttle: 0, brake: 0 },
      scenario: { name: '手动放置', mu: 0.9 },
      obstacles: [],
      collision: { hit: false, obstacle_id: null }
    };
    this._onSimData(demoData);
  }

  // ========== 数据处理 ==========

  /** 收到仿真数据时更新所有组件 */
  _onSimData(data) {
    // 更新 Canvas（含障碍物和碰撞状态）
    this.vehicleCanvas.update(data);
    // 更新图表
    this.charts.update(data);
    // 更新传感器面板
    this._updateSensorPanel(data);
    // 更新底盘系统徽章与状态
    this._updateSystemsPanel(data);
    // 更新 ESP 状态灯
    this.controls.updateESPState(data.esp);
    // 更新底部栏
    this._updateFooter(data);
    // 更新 HUD（含碰撞提示）
    this._updateHUD(data);
  }

  /** 更新底盘系统徽章（装配状态）+ 状态行（实时介入）+ 车轮抱死标记 */
  _updateSystemsPanel(data) {
    const era = data.era;
    const systems = data.systems || {};
    const wheels = data.wheels || {};

    // 徽章：装配的系统亮起，未装配的置灰
    if (era && era.systems) {
      for (const [key, el] of Object.entries(this.sysEls.badges)) {
        if (el) el.dataset.on = era.systems[key] ? 'true' : 'false';
      }
    }

    // 状态行：哪个系统正在介入
    const active = [];
    if (systems.abs_active) active.push('ABS 调压中');
    if (systems.asr_active) active.push('ASR 削减油门');
    const esp = data.esp || {};
    if (esp.active) active.push(`ESP 制动 ${esp.target_wheel || ''}`);
    if (Math.abs(systems.tv_moment || 0) > 50) active.push('扭矩矢量分配中');
    const anyLock = Object.values(wheels).some(w => w && w.locked);
    if (anyLock) active.push('车轮抱死!');
    if (this.sysEls.statusLine) {
      this.sysEls.statusLine.textContent = active.length ? active.join(' · ') : '待命';
      this.sysEls.statusLine.classList.toggle('sys-status-line--alert', anyLock || !!systems.asr_active || !!systems.abs_active);
    }

    // 车轮抱死标记
    for (const [key, el] of Object.entries(this.sysEls.locks)) {
      const w = wheels[key];
      if (el) el.style.display = (w && w.locked) ? 'inline-block' : 'none';
    }
  }

  /** 更新右栏传感器数值 */
  _updateSensorPanel(data) {
    const w = data.wheels || {};
    const s = data.sensors || {};
    const esp = data.esp || {};

    // 四轮轮速
    this._setText(this.sensorEls.wheelFL, this._fmt(w.fl?.speed, 1));
    this._setText(this.sensorEls.wheelFR, this._fmt(w.fr?.speed, 1));
    this._setText(this.sensorEls.wheelRL, this._fmt(w.rl?.speed, 1));
    this._setText(this.sensorEls.wheelRR, this._fmt(w.rr?.speed, 1));

    // 横摆角速度
    this._setText(this.sensorEls.yawRate, this._fmt(s.yaw_rate, 2));

    // 侧向加速度
    this._setText(this.sensorEls.latAccel, this._fmt(s.lateral_accel, 2));

    // 方向盘转角
    this._setText(this.sensorEls.steeringAngle, this._fmt(s.steering_angle, 1));

    // 制动压力柱状图
    const maxPressure = 150; // bar 满量程
    this._updateBrakeBar('fl', w.fl?.pressure || 0, maxPressure);
    this._updateBrakeBar('fr', w.fr?.pressure || 0, maxPressure);
    this._updateBrakeBar('rl', w.rl?.pressure || 0, maxPressure);
    this._updateBrakeBar('rr', w.rr?.pressure || 0, maxPressure);

    // ESP 横摆力矩
    this._setText(this.sensorEls.espMoment, this._fmt(esp.mz_applied, 0));

    // ESP 目标轮
    const targetEl = this.sensorEls.espTarget;
    if (targetEl) {
      if (esp.active && esp.target_wheel) {
        targetEl.textContent = `制动车轮: ${esp.target_wheel}`;
        targetEl.style.display = 'block';
      } else {
        targetEl.textContent = '无介入';
        targetEl.style.opacity = '0.5';
      }
    }

    // 力分配高亮
    this._updateForceDistribution(esp);
  }

  /** 更新制动压力柱 */
  _updateBrakeBar(wheel, pressure, max) {
    const percent = Math.min(100, (pressure / max) * 100);
    const fillEl = document.getElementById(`brake-fill-${wheel}`);
    const valEl = document.getElementById(`brake-val-${wheel}`);
    if (fillEl) fillEl.style.height = percent + '%';
    if (valEl) valEl.textContent = pressure.toFixed(0) + ' bar';
  }

  /** 更新力分配可视化 */
  _updateForceDistribution(esp) {
    const positions = ['fl', 'fr', 'rl', 'rr'];
    const nameMap = { 'FL': 'fl', 'FR': 'fr', 'RL': 'rl', 'RR': 'rr' };

    positions.forEach(pos => {
      const el = this.sensorEls['force' + pos.toUpperCase()];
      if (el) {
        const isTarget = esp.active && nameMap[esp.target_wheel] === pos;
        el.classList.toggle('force-wheel--active', isTarget);
      }
    });
  }

  /** 更新底部状态栏 */
  _updateFooter(data) {
    const time = data.time || 0;
    const vx = data.vehicle?.vx || 0;
    const speed = Math.abs(vx * 3.6);
    const esp = data.esp || {};

    this._setText(this.footerEls.time, `T: ${time.toFixed(2)}s`);
    this._setText(this.footerEls.speed, `${speed.toFixed(1)} km/h`);
    this._setText(this.footerEls.scenario, data.era?.name || '手动放置');

    // ESP 状态（浅色主题配色）
    const espEl = this.footerEls.espStatus;
    if (espEl) {
      if (!esp.enabled) {
        espEl.textContent = 'ESP OFF';
        espEl.style.color = '#d94f4f';
      } else if (esp.active) {
        espEl.textContent = 'ESP 介入中';
        espEl.style.color = '#e07a3f';
      } else {
        espEl.textContent = 'ESP ON';
        espEl.style.color = '#5a9e6f';
      }
    }
  }

  /** 更新 Canvas HUD（含碰撞提示与时代信息） */
  _updateHUD(data) {
    const vx = data.vehicle?.vx || 0;
    const speed = Math.abs(vx * 3.6);
    const scenario = data.scenario || {};
    const collision = data.collision || {};

    this._setText(this.hudEls.speed, `${speed.toFixed(0)} km/h`);
    this._setText(this.hudEls.scenario, data.era?.name || '手动放置模式');
    this._setText(this.hudEls.mu, `μ = ${(scenario.mu || 0.9).toFixed(1)}`);

    // 碰撞提示
    if (this.hudEls.collision) {
      if (collision.hit) {
        this.hudEls.collision.textContent = '⚠ 碰撞!';
        this.hudEls.collision.style.display = 'block';
      } else {
        this.hudEls.collision.style.display = 'none';
      }
    }
  }

  // ========== 控制发送 ==========

  /** 输入变化时发送控制指令（节流），steering 范围 -1080~1080 */
  _onInputChange(inputs) {
    const now = Date.now();
    if (now - this._lastSendTime < this._sendInterval) return;
    this._lastSendTime = now;

    this._postAPI('/api/control', {
      steering: inputs.steering,   // -1080 ~ +1080 度
      throttle: inputs.throttle,
      brake: inputs.brake
    });
  }

  /** 回到原点：车辆传送回 (0,0)，速度归零，不改变障碍物 */
  _onResetPosition() {
    this._postAPI('/api/reset_position', {});
    // 清除轨迹历史（车辆位置跳变，轨迹线不再连续）
    this.vehicleCanvas.reset();
  }

  /** ESP 开关 */
  _onESPToggle(enabled) {
    this._postAPI('/api/esp_toggle', { enabled });
  }

  /** 路面选择 */
  _onMuChange(value) {
    this._postAPI('/api/mu', { value });
  }

  // ========== 技术世代 ==========

  /** 初始化时代选择器：拉取当前时代并同步下拉框 */
  async _initEra() {
    const select = document.getElementById('era-select');
    if (!select) return;
    select.addEventListener('change', (e) => this._onEraChange(e.target.value));
    try {
      const resp = await fetch('/api/eras');
      const result = await resp.json();
      if (result.status === 'ok' && result.current) {
        select.value = result.current;
      }
    } catch (e) {
      // 离线模式下静默失败，保留 HTML 默认选中
    }
  }

  /** 切换时代：后端自动配好系统组合+推荐路面+回原点，前端同步控件状态 */
  async _onEraChange(eraId) {
    try {
      const resp = await fetch('/api/era', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ era: eraId })
      });
      const result = await resp.json();
      if (result.status === 'ok') {
        // 同步 ESP 开关 UI
        const espToggle = document.getElementById('esp-toggle-input');
        if (espToggle) espToggle.checked = result.esp_enabled;
        // 同步路面按钮 UI（后端已应用推荐 mu）
        document.querySelectorAll('.road-btn').forEach(btn => {
          btn.classList.toggle('active', Math.abs(parseFloat(btn.dataset.mu) - result.mu) < 0.01);
        });
        // 车辆回原点，清除轨迹历史
        this.vehicleCanvas.reset();
        this.charts.reset();
      }
    } catch (e) {
      console.warn('[ESP] 时代切换失败:', e.message);
    }
  }

  /** 暂停/播放 */
  _sendPause(paused) {
    this._postAPI('/api/pause', { paused });
  }

  /** 重置 */
  _sendReset() {
    this._postAPI('/api/reset', {});
    this.charts.reset();
    this.vehicleCanvas.reset();
  }

  /** 慢动作 */
  _sendSlowmo(rate) {
    this._postAPI('/api/slowmo', { rate });
  }

  // ========== 顶部控件 ==========

  _initHeaderControls() {
    // 回到原点按钮
    const resetPositionBtn = document.getElementById('reset-position-btn');
    if (resetPositionBtn) {
      resetPositionBtn.addEventListener('click', () => this._onResetPosition());
    }

    // ESP 开关
    const espToggle = document.getElementById('esp-toggle-input');
    if (espToggle) {
      espToggle.addEventListener('change', (e) => {
        this._onESPToggle(e.target.checked);
      });
    }

    // 路面选择按钮
    const roadBtns = document.querySelectorAll('.road-btn');
    roadBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        roadBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const mu = parseFloat(btn.dataset.mu);
        this._onMuChange(mu);
      });
    });
  }

  _initFooter() {
    // 底部重置按钮
    const footerReset = document.getElementById('footer-reset-btn');
    if (footerReset) {
      footerReset.addEventListener('click', () => this._sendReset());
    }
  }

  _bindControlEvents() {
    // 全局快捷键（避免与 WASD 冲突）
    document.addEventListener('keydown', (e) => {
      // Space 暂停/播放
      if (e.code === 'Space' && !e.target.matches('input, select, textarea')) {
        e.preventDefault();
        this.controls._togglePlay();
      }
      // R 重置
      if (e.code === 'KeyR' && e.ctrlKey) {
        e.preventDefault();
        this._sendReset();
      }
    });
  }

  // ========== 工具方法 ==========

  /** POST API 请求 */
  async _postAPI(url, body) {
    try {
      await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      });
    } catch (e) {
      // 离线模式下静默失败
      if (this.connected) {
        console.warn(`[ESP] API 请求失败: ${url}`, e.message);
      }
    }
  }

  /** 更新连接状态指示 */
  _updateConnectionStatus(connected) {
    const dot = document.getElementById('connection-dot');
    const text = this.footerEls.connection;
    if (dot) {
      dot.className = connected ? 'footer__dot' : 'footer__dot footer__dot--disconnected';
    }
    if (text) {
      text.textContent = connected ? '已连接' : '离线模式';
    }
  }

  /** 安全设置文本 */
  _setText(el, value) {
    if (el) el.textContent = value;
  }

  /** 数字格式化 */
  _fmt(value, decimals = 1) {
    if (value === undefined || value === null) return '--';
    return Number(value).toFixed(decimals);
  }
}

// ========== 应用启动 ==========
document.addEventListener('DOMContentLoaded', () => {
  window.espApp = new ESPApp();
});
