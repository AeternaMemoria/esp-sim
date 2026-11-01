/**
 * Controls — ESP 教学模拟：驾驶输入控件
 * 功能：方向盘拖拽(±1080°)、油门/刹车滑块(松手回弹)、ESP状态灯、播放控制
 */
class Controls {
  /**
   * @param {HTMLElement} containerElement — 左栏容器
   * @param {Function} onInputChange — 输入变化回调 ({steering, throttle, brake})
   */
  constructor(containerElement, onInputChange) {
    this.container = containerElement;
    this.onInputChange = onInputChange;

    // 当前输入值
    this.inputs = {
      steering: 0,    // -1080 ~ +1080 度
      throttle: 0,    // 0 ~ 1
      brake: 0        // 0 ~ 1
    };

    // 方向盘最大转角
    this.MAX_STEERING = 1080;

    // 键盘状态
    this._keys = { a: false, d: false, w: false, s: false };
    this._steeringVelocity = 0;

    // 方向盘拖拽状态（增量累加法，支持 ±1080° 多圈连续旋转）
    this._dragging = false;
    this._lastPointerAngle = null; // 上一帧鼠标角度（度），null 表示尚未开始
    this._wheelAngle = 0;

    // 方向盘到达极限时的状态
    this._atLimit = false;       // 是否处于极限
    this._limitFlashTimer = null;

    // 自动回中
    this._autoCenter = true;

    // ESP 状态
    this._espState = 'active';

    // 刹车拖拽状态（用于松手回弹）
    this._brakeDragging = false;

    // 构建 UI
    this._buildDOM();
    this._bindEvents();
    this._startUpdateLoop();
  }

  /** 构建控件 DOM */
  _buildDOM() {
    this.container.innerHTML = `
      <!-- 方向盘 -->
      <div class="panel__section">
        <div class="panel__section-title">方向盘</div>
        <div class="steering-container">
          <div class="steering-wheel" id="steering-wheel">
            <canvas id="steering-canvas" width="280" height="280"></canvas>
          </div>
          <div class="steering-value" id="steering-value">0°</div>
          <div class="steering-hint">拖拽旋转 | A/D 键 | ±1080°</div>
        </div>
      </div>

      <!-- 油门 & 刹车 -->
      <div class="panel__section">
        <div class="panel__section-title">踏板</div>
        <div class="pedals-container">
          <div class="pedal-group">
            <div class="pedal-label">油门</div>
            <div class="pedal-slider-wrap" id="throttle-track">
              <div class="pedal-fill pedal-fill--throttle" id="throttle-fill"></div>
            </div>
            <div class="pedal-value" id="throttle-value">0%</div>
            <div class="pedal-key-hint">W</div>
          </div>
          <div class="pedal-group">
            <div class="pedal-label">刹车</div>
            <div class="pedal-slider-wrap" id="brake-track">
              <div class="pedal-fill pedal-fill--brake" id="brake-fill"></div>
            </div>
            <div class="pedal-value" id="brake-value">0%</div>
            <div class="pedal-key-hint">S</div>
          </div>
        </div>
      </div>

      <!-- ESP 状态灯 -->
      <div class="panel__section">
        <div class="panel__section-title">ESP 状态</div>
        <div class="esp-status-light">
          <div class="esp-light esp-light--active" id="esp-light"></div>
          <div class="esp-status-text" id="esp-status-text">ESP 就绪</div>
        </div>
      </div>

      <!-- 播放控制 -->
      <div class="panel__section">
        <div class="panel__section-title">播放控制</div>
        <div class="playback-controls">
          <div class="btn-row">
            <button class="btn btn--primary" id="btn-play" title="暂停/播放">
              <span id="play-icon">⏸</span> <span id="play-text">暂停</span>
            </button>
            <button class="btn btn--danger" id="btn-reset" title="重置">↺ 重置</button>
          </div>
          <div class="btn-row">
            <button class="btn btn--speed" data-rate="0.25">0.25x</button>
            <button class="btn btn--speed" data-rate="0.5">0.5x</button>
            <button class="btn btn--speed active" data-rate="1">1x</button>
          </div>
        </div>
      </div>
    `;

    // 缓存 DOM 引用
    this.els = {
      steeringCanvas: document.getElementById('steering-canvas'),
      steeringWheel: document.getElementById('steering-wheel'),
      steeringValue: document.getElementById('steering-value'),
      throttleFill: document.getElementById('throttle-fill'),
      throttleValue: document.getElementById('throttle-value'),
      throttleTrack: document.getElementById('throttle-track'),
      brakeFill: document.getElementById('brake-fill'),
      brakeValue: document.getElementById('brake-value'),
      brakeTrack: document.getElementById('brake-track'),
      espLight: document.getElementById('esp-light'),
      espStatusText: document.getElementById('esp-status-text'),
      btnPlay: document.getElementById('btn-play'),
      playIcon: document.getElementById('play-icon'),
      playText: document.getElementById('play-text'),
      btnReset: document.getElementById('btn-reset'),
      speedBtns: this.container.querySelectorAll('.btn--speed')
    };

    // 初始化方向盘 canvas
    this._steeringCtx = this.els.steeringCanvas.getContext('2d');
    this._drawSteeringWheel();
  }

  /** 绑定所有交互事件 */
  _bindEvents() {
    // --- 方向盘拖拽 ---
    const wheel = this.els.steeringWheel;

    wheel.addEventListener('pointerdown', (e) => {
      this._dragging = true;
      this._autoCenter = false;
      wheel.setPointerCapture(e.pointerId);
      // 只记录起始鼠标角度，不重置 _wheelAngle，保持当前位置
      this._lastPointerAngle = this._getPointerAngle(e);
      e.preventDefault();
    });

    wheel.addEventListener('pointermove', (e) => {
      if (!this._dragging) return;
      const currentAngle = this._getPointerAngle(e);
      if (this._lastPointerAngle === null) {
        this._lastPointerAngle = currentAngle;
        return;
      }

      // 计算相邻两帧鼠标角度增量，并修正跨 ±180° 线的跳变
      let delta = currentAngle - this._lastPointerAngle;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      this._lastPointerAngle = currentAngle;

      // 增量累加到当前方向盘角度（屏幕顺时针拖拽 = 右转正角度）
      const newAngle = this._wheelAngle + delta;
      // 限制范围 -1080 ~ +1080，到达极限时触发闪烁
      const clamped = Math.max(-this.MAX_STEERING, Math.min(this.MAX_STEERING, newAngle));
      if (clamped !== newAngle) {
        this._triggerLimitFlash();
      }
      this._wheelAngle = clamped;
      this.inputs.steering = clamped;
      this._updateSteeringDisplay();
      this._emitChange();
    });

    wheel.addEventListener('pointerup', () => {
      this._dragging = false;
      this._lastPointerAngle = null;
      this._autoCenter = true;
    });

    wheel.addEventListener('pointercancel', () => {
      this._dragging = false;
      this._lastPointerAngle = null;
      this._autoCenter = true;
    });

    // --- 油门滑块拖拽（松手归零） ---
    this._bindPedalDrag(this.els.throttleTrack, 'throttle', true);
    // --- 刹车滑块拖拽（松手归零） ---
    this._bindPedalDrag(this.els.brakeTrack, 'brake', true);

    // --- 键盘事件 ---
    document.addEventListener('keydown', (e) => this._onKeyDown(e));
    document.addEventListener('keyup', (e) => this._onKeyUp(e));

    // --- 播放控制 ---
    this.els.btnPlay.addEventListener('click', () => this._togglePlay());
    this.els.btnReset.addEventListener('click', () => this._onReset());

    this.els.speedBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        this.els.speedBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const rate = parseFloat(btn.dataset.rate);
        this._onSpeedChange(rate);
      });
    });
  }

  /**
   * 绑定踏板拖拽
   * @param {HTMLElement} trackEl — 滑块轨道元素
   * @param {string} type — 'throttle' 或 'brake'
   * @param {boolean} autoReturn — 松手后是否自动归零
   */
  _bindPedalDrag(trackEl, type, autoReturn = true) {
    let dragging = false;

    const updateFromPointer = (e) => {
      const rect = trackEl.getBoundingClientRect();
      const y = e.clientY - rect.top;
      const ratio = 1 - Math.max(0, Math.min(1, y / rect.height));
      this.inputs[type] = ratio;
      this._updatePedalDisplay(type);
      this._emitChange();
    };

    trackEl.addEventListener('pointerdown', (e) => {
      dragging = true;
      trackEl.setPointerCapture(e.pointerId);
      updateFromPointer(e);
      e.preventDefault();
    });

    trackEl.addEventListener('pointermove', (e) => {
      if (dragging) updateFromPointer(e);
    });

    const onRelease = () => {
      if (!dragging) return;
      dragging = false;
      // 松手自动回弹到 0
      if (autoReturn) {
        this.inputs[type] = 0;
        this._updatePedalDisplay(type);
        this._emitChange();
      }
    };

    trackEl.addEventListener('pointerup', onRelease);
    trackEl.addEventListener('pointercancel', onRelease);
  }

  /** 键盘按下 */
  _onKeyDown(e) {
    const key = e.key.toLowerCase();
    if (['a', 'd', 'w', 's'].includes(key)) {
      if (this._keys[key]) return; // 防止重复触发
      this._keys[key] = true;
      e.preventDefault();
    }
  }

  /** 键盘松开 */
  _onKeyUp(e) {
    const key = e.key.toLowerCase();
    if (['a', 'd', 'w', 's'].includes(key)) {
      this._keys[key] = false;
      // 松开 W 时油门归零
      if (key === 'w') {
        this.inputs.throttle = 0;
        this._updatePedalDisplay('throttle');
        this._emitChange();
      }
      // 松开 S 时刹车归零（自动回弹）
      if (key === 's') {
        this.inputs.brake = 0;
        this._updatePedalDisplay('brake');
        this._emitChange();
      }
      // 松开 A/D 时恢复自动回中
      if (key === 'a' || key === 'd') {
        if (!this._keys.a && !this._keys.d) {
          this._autoCenter = true;
        }
      }
    }
  }

  /** 定时更新循环（处理键盘持续输入） */
  _startUpdateLoop() {
    this._loopId = setInterval(() => {
      let changed = false;

      // 方向盘键盘控制（±1080°范围）
      const steerSpeed = 9; // 度/帧 — 9×30fps=270°/s → 0到1080°约需4秒
      if (this._keys.a) {
        const newVal = this.inputs.steering - steerSpeed;
        if (newVal <= -this.MAX_STEERING) {
          this.inputs.steering = -this.MAX_STEERING;
          this._triggerLimitFlash();
        } else {
          this.inputs.steering = newVal;
        }
        this._wheelAngle = this.inputs.steering;
        this._autoCenter = false;
        changed = true;
      }
      if (this._keys.d) {
        const newVal = this.inputs.steering + steerSpeed;
        if (newVal >= this.MAX_STEERING) {
          this.inputs.steering = this.MAX_STEERING;
          this._triggerLimitFlash();
        } else {
          this.inputs.steering = newVal;
        }
        this._wheelAngle = this.inputs.steering;
        this._autoCenter = false;
        changed = true;
      }

      // 自动回中
      if (this._autoCenter && !this._keys.a && !this._keys.d && Math.abs(this.inputs.steering) > 0.5) {
        this.inputs.steering *= 0.85; // 指数衰减回中
        if (Math.abs(this.inputs.steering) < 0.5) this.inputs.steering = 0;
        this._wheelAngle = this.inputs.steering;
        changed = true;
      }

      // 油门键盘控制
      if (this._keys.w) {
        this.inputs.throttle = Math.min(1, this.inputs.throttle + 0.04);
        this._updatePedalDisplay('throttle');
        changed = true;
      }

      // 刹车键盘控制（按住增加，松开时在 _onKeyUp 中归零）
      if (this._keys.s) {
        this.inputs.brake = Math.min(1, this.inputs.brake + 0.06);
        this._updatePedalDisplay('brake');
        changed = true;
      }

      if (changed) {
        this._updateSteeringDisplay();
        this._emitChange();
      }
    }, 1000 / 30); // 30fps 更新
  }

  /** 触发方向盘极限闪烁反馈 */
  _triggerLimitFlash() {
    if (this._limitFlashTimer) return; // 防止重复触发
    this._atLimit = true;
    this.els.steeringWheel.classList.add('steering-wheel--hit');
    this._limitFlashTimer = setTimeout(() => {
      this.els.steeringWheel.classList.remove('steering-wheel--hit');
      this._limitFlashTimer = null;
      // 只在不再处于极限位置时清除红色状态
      if (Math.abs(this.inputs.steering) < this.MAX_STEERING) {
        this._atLimit = false;
      }
    }, 300);
  }

  /** 获取指针相对于方向盘中心的角度（度，范围 [-180, 180]，仅供增量计算使用） */
  _getPointerAngle(e) {
    const rect = this.els.steeringWheel.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const dx = e.clientX - cx;
    const dy = e.clientY - cy;
    return Math.atan2(dy, dx) * (180 / Math.PI);
  }

  /** 更新方向盘显示 */
  _updateSteeringDisplay() {
    const val = Math.round(this.inputs.steering);
    this.els.steeringValue.textContent = `${val}°`;

    // 到达极限时数值变红
    const atLimit = Math.abs(this.inputs.steering) >= this.MAX_STEERING;
    this.els.steeringValue.classList.toggle('steering-value--limit', atLimit);

    this._drawSteeringWheel();
  }

  /** 绘制方向盘图形 */
  _drawSteeringWheel() {
    const canvas = this.els.steeringCanvas;
    const ctx = this._steeringCtx;
    const size = 280;
    const cx = size / 2;
    const cy = size / 2;
    const outerR = 120;
    const innerR = 100;
    const hubR = 22;

    // 判断是否处于极限位置（变红）
    const atLimit = Math.abs(this.inputs.steering) >= this.MAX_STEERING;

    ctx.clearRect(0, 0, size, size);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((this._wheelAngle * Math.PI) / 180);

    // 外圈颜色：正常为暖灰，极限时变红
    const rimColor = atLimit ? '#d94f4f' : '#6b5e52';
    const rimInner = atLimit ? '#c04040' : '#4a3f35';

    // 外圈
    ctx.beginPath();
    ctx.arc(0, 0, outerR, 0, Math.PI * 2);
    ctx.strokeStyle = rimColor;
    ctx.lineWidth = 18;
    ctx.stroke();

    // 握把皮革质感
    ctx.beginPath();
    ctx.arc(0, 0, outerR, 0, Math.PI * 2);
    ctx.strokeStyle = rimInner;
    ctx.lineWidth = 14;
    ctx.stroke();

    // 握把高光
    ctx.beginPath();
    ctx.arc(0, 0, outerR, -Math.PI * 0.8, -Math.PI * 0.2);
    ctx.strokeStyle = 'rgba(255,255,255,0.12)';
    ctx.lineWidth = 14;
    ctx.stroke();

    // 极限时红色发光
    if (atLimit) {
      ctx.beginPath();
      ctx.arc(0, 0, outerR, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(217, 79, 79, 0.3)';
      ctx.lineWidth = 24;
      ctx.stroke();
    }

    // 三辐条
    ctx.strokeStyle = atLimit ? '#d94f4f' : '#6b5e52';
    ctx.lineWidth = 8;
    ctx.lineCap = 'round';
    for (let i = 0; i < 3; i++) {
      const angle = (i * 120 - 90) * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo(Math.cos(angle) * hubR, Math.sin(angle) * hubR);
      ctx.lineTo(Math.cos(angle) * (innerR - 4), Math.sin(angle) * (innerR - 4));
      ctx.stroke();
    }

    // 中心轮毂
    ctx.beginPath();
    ctx.arc(0, 0, hubR, 0, Math.PI * 2);
    ctx.fillStyle = atLimit ? '#f5ebe0' : '#f9f5f0';
    ctx.fill();
    ctx.strokeStyle = atLimit ? '#d94f4f' : '#6b5e52';
    ctx.lineWidth = 2;
    ctx.stroke();

    // 中心 ESP 字样
    ctx.fillStyle = atLimit ? '#d94f4f' : '#7a7a7a';
    ctx.font = 'bold 10px Orbitron, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('ESP', 0, 0);

    // 顶部标记（12点方向）
    ctx.beginPath();
    ctx.arc(0, -outerR, 4, 0, Math.PI * 2);
    ctx.fillStyle = atLimit ? '#d94f4f' : '#e07a3f';
    ctx.fill();

    ctx.restore();

    // 固定标记（不随方向盘转动）— 12点钟位置指示线
    ctx.beginPath();
    ctx.moveTo(cx, cy - outerR - 14);
    ctx.lineTo(cx, cy - outerR - 6);
    ctx.strokeStyle = atLimit ? '#d94f4f' : '#e07a3f';
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  /** 更新踏板显示 */
  _updatePedalDisplay(type) {
    const value = this.inputs[type];
    const percent = Math.round(value * 100);

    if (type === 'throttle') {
      this.els.throttleFill.style.height = percent + '%';
      this.els.throttleValue.textContent = percent + '%';
    } else {
      this.els.brakeFill.style.height = percent + '%';
      this.els.brakeValue.textContent = percent + '%';
    }
  }

  /** 发射输入变化事件 */
  _emitChange() {
    if (this.onInputChange) {
      this.onInputChange({ ...this.inputs });
    }
  }

  // ========== 公共 API ==========

  /** 获取当前输入 */
  getInputs() {
    return { ...this.inputs };
  }

  /**
   * 更新 ESP 状态灯
   * @param {Object} espData — {enabled, active, intensity}
   */
  updateESPState(espData) {
    if (!espData) return;

    const light = this.els.espLight;
    const text = this.els.espStatusText;

    // 清除所有状态类
    light.className = 'esp-light';

    if (!espData.enabled) {
      light.classList.add('esp-light--off');
      text.textContent = 'ESP 已关闭';
      text.style.color = '#a09a94';
    } else if (espData.active && espData.intensity >= 0.5) {
      light.classList.add('esp-light--danger');
      text.textContent = `ESP 重度介入 (${(espData.intensity * 100).toFixed(0)}%)`;
      text.style.color = '#d94f4f';
    } else if (espData.active) {
      light.classList.add('esp-light--warning');
      text.textContent = `ESP 轻度介入 (${(espData.intensity * 100).toFixed(0)}%)`;
      text.style.color = '#e07a3f';
    } else {
      light.classList.add('esp-light--active');
      text.textContent = 'ESP 就绪';
      text.style.color = '#5a9e6f';
    }
  }

  /** 播放/暂停切换 */
  _togglePlay() {
    const isPaused = this.els.playIcon.textContent === '▶';
    if (isPaused) {
      this.els.playIcon.textContent = '⏸';
      this.els.playText.textContent = '暂停';
      this._onPlayStateChange(false);
    } else {
      this.els.playIcon.textContent = '▶';
      this.els.playText.textContent = '播放';
      this._onPlayStateChange(true);
    }
  }

  /** 播放状态变化回调 */
  _onPlayStateChange(paused) {
    if (this.onPlayStateChange) this.onPlayStateChange(paused);
  }

  /** 重置回调 */
  _onReset() {
    if (this.onReset) this.onReset();
  }

  /** 速度变化回调 */
  _onSpeedChange(rate) {
    if (this.onSpeedChange) this.onSpeedChange(rate);
  }

  /** 销毁 */
  destroy() {
    if (this._loopId) clearInterval(this._loopId);
    if (this._limitFlashTimer) clearTimeout(this._limitFlashTimer);
  }
}
