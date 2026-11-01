/**
 * VehicleCanvas — ESP 教学模拟：车辆俯视图动画
 * 功能：绘制路面、车辆线框、四轮轨迹线、障碍物、碰撞闪烁、ESP介入
 * 配色适配暖色浅色主题
 */
class VehicleCanvas {
  constructor(canvasElement) {
    this.canvas = canvasElement;
    this.ctx = canvasElement.getContext('2d');

    // 高 DPI 适配
    this.dpr = window.devicePixelRatio || 1;
    this._resize();

    // === 四轮轨迹数据 ===
    // 车辆几何参数（米）
    this._lf = 1.35;   // 前轴到质心距离
    this._lr = 1.35;   // 后轴到质心距离
    this._tw2 = 0.775; // 半轮距

    // 每个轮独立轨迹（最多保留150个点）
    this.maxTrailLength = 150;
    this.trails = { FL: [], FR: [], RL: [], RR: [] };

    // 轨迹颜色：前轮暖色系，后轮冷色系
    this.trailColors = {
      FL: { r: 224, g: 122, b: 63 },   // 橙
      FR: { r: 218, g: 165, b: 32 },   // 黄/金
      RL: { r: 74, g: 143, b: 189 },   // 蓝
      RR: { r: 61, g: 143, b: 168 }    // 青
    };

    // === 障碍物数据 ===
    this.obstacles = [];  // [{id, x, y, radius}]
    this.collision = { hit: false, obstacle_id: null };

    // 放置动画效果队列 [{x, y, startTime}]
    this._placeEffects = [];

    // 当前状态
    this.state = {
      x: 0, y: 0, psi: 0,
      vx: 0, vy: 0, omega_z: 0,
      steering: 0,
      espEnabled: true,
      espActive: false,
      espIntensity: 0,
      targetWheel: null,
      mu: 0.9,
      wheelLocked: { FL: false, FR: false, RL: false, RR: false }  // 车轮抱死状态
    };

    // 动画计时
    this._flashPhase = 0;
    this._animFrameId = null;
    this._collisionFlash = 0; // 碰撞闪烁强度

    // 路面标记线间距（米）
    this._roadMarkSpacing = 5;

    // 比例尺：像素/米
    this.scale = 8;

    // 监听窗口 resize
    this._resizeObserver = new ResizeObserver(() => this._resize());
    this._resizeObserver.observe(canvasElement.parentElement);

    // 绑定点击删除障碍物
    this.canvas.addEventListener('click', (e) => this._onCanvasClick(e));

    // 启动动画循环
    this._startLoop();
  }

  /** 自适应画布尺寸 */
  _resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    this.width = rect.width;
    this.height = rect.height;
    this.canvas.width = this.width * this.dpr;
    this.canvas.height = this.height * this.dpr;
    this.canvas.style.width = this.width + 'px';
    this.canvas.style.height = this.height + 'px';
    this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
  }

  /** 接收 WebSocket 数据并更新状态 */
  update(data) {
    if (!data) return;

    const v = data.vehicle || {};
    const esp = data.esp || {};
    const inputs = data.inputs || {};
    const scenario = data.scenario || {};

    this.state.x = v.x || 0;
    this.state.y = v.y || 0;
    this.state.psi = v.psi || 0;
    this.state.vx = v.vx || 0;
    this.state.vy = v.vy || 0;
    this.state.omega_z = v.omega_z || 0;
    this.state.steering = inputs.steering || 0;
    this.state.espEnabled = esp.enabled !== false;
    this.state.espActive = esp.active || false;
    this.state.espIntensity = esp.intensity || 0;
    this.state.targetWheel = esp.target_wheel || null;
    this.state.mu = scenario.mu || 0.9;

    // 车轮抱死状态（裸车时代猛刹的关键视觉反馈）
    const wheels = data.wheels || {};
    this.state.wheelLocked = {
      FL: !!(wheels.fl && wheels.fl.locked),
      FR: !!(wheels.fr && wheels.fr.locked),
      RL: !!(wheels.rl && wheels.rl.locked),
      RR: !!(wheels.rr && wheels.rr.locked)
    };

    // 更新障碍物数据
    if (data.obstacles) {
      this.obstacles = data.obstacles;
    }

    // 更新碰撞状态
    if (data.collision) {
      this.collision = data.collision;
      if (data.collision.hit) {
        this._collisionFlash = 1.0; // 触发碰撞闪烁
      }
    }

    // 计算四轮世界坐标并添加轨迹点
    this._updateWheelTrails();
  }

  /** 计算四轮位置并记录轨迹 */
  _updateWheelTrails() {
    const { x, y, psi } = this.state;
    const cosP = Math.cos(psi);
    const sinP = Math.sin(psi);
    const lf = this._lf, lr = this._lr, tw2 = this._tw2;

    // 四轮世界坐标
    const wheels = {
      FL: { x: x + lf * cosP - tw2 * sinP, y: y + lf * sinP + tw2 * cosP },
      FR: { x: x + lf * cosP + tw2 * sinP, y: y + lf * sinP - tw2 * cosP },
      RL: { x: x - lr * cosP - tw2 * sinP, y: y - lr * sinP + tw2 * cosP },
      RR: { x: x - lr * cosP + tw2 * sinP, y: y - lr * sinP - tw2 * cosP }
    };

    // 添加到各自轨迹
    for (const [key, pos] of Object.entries(wheels)) {
      this.trails[key].push(pos);
      if (this.trails[key].length > this.maxTrailLength) {
        this.trails[key].shift();
      }
    }
  }

  /** 清空轨迹和状态 */
  reset() {
    this.trails = { FL: [], FR: [], RL: [], RR: [] };
    this.obstacles = [];
    this.collision = { hit: false, obstacle_id: null };
    this._collisionFlash = 0;
    this.state = {
      x: 0, y: 0, psi: 0,
      vx: 0, vy: 0, omega_z: 0,
      steering: 0,
      espEnabled: true,
      espActive: false,
      espIntensity: 0,
      targetWheel: null,
      mu: 0.9,
      wheelLocked: { FL: false, FR: false, RL: false, RR: false }
    };
  }

  /** Canvas 点击事件：点中障碍物→删除，点击空白→放置新障碍物 */
  _onCanvasClick(e) {
    const rect = this.canvas.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;

    // 屏幕坐标转世界坐标（画布中心 = 车辆位置）
    const worldX = (screenX - this.width / 2) / this.scale + this.state.x;
    const worldY = -(screenY - this.height / 2) / this.scale + this.state.y;

    // 检查是否点中障碍物
    for (const obs of this.obstacles) {
      const dx = worldX - obs.x;
      const dy = worldY - obs.y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < obs.radius) {
        // 点中障碍物 → 删除
        this._deleteObstacle(obs.id);
        return;
      }
    }

    // 点击空白区域 → 放置新障碍物
    this._addObstacle(worldX, worldY);
  }

  /** 发送删除障碍物 API */
  async _deleteObstacle(id) {
    try {
      await fetch('/api/obstacle/delete', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ id })
      });
      // 本地立即移除（乐观更新）
      this.obstacles = this.obstacles.filter(o => o.id !== id);
    } catch (e) {
      console.warn('[VehicleCanvas] 删除障碍物失败:', e.message);
    }
  }

  /** 发送放置障碍物 API，并触发视觉反馈动画 */
  async _addObstacle(worldX, worldY) {
    // 添加放置动画效果
    this._placeEffects.push({ x: worldX, y: worldY, startTime: performance.now() });

    try {
      const resp = await fetch('/api/obstacle/add', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x: worldX, y: worldY })
      });
      const result = await resp.json();
      if (result.status === 'ok') {
        // 乐观更新：本地立即添加障碍物显示
        this.obstacles.push({
          id: result.id,
          x: worldX,
          y: worldY,
          radius: 0.75  // 默认显示半径，WebSocket 推送后会更新为实际值
        });
      }
    } catch (e) {
      console.warn('[VehicleCanvas] 放置障碍物失败:', e.message);
    }
  }

  /** 动画循环 */
  _startLoop() {
    const loop = () => {
      this._flashPhase += 0.08;
      // 碰撞闪烁衰减
      if (this._collisionFlash > 0) {
        this._collisionFlash *= 0.94;
        if (this._collisionFlash < 0.01) this._collisionFlash = 0;
      }
      this._draw();
      this._animFrameId = requestAnimationFrame(loop);
    };
    loop();
  }

  /** 销毁 */
  destroy() {
    if (this._animFrameId) cancelAnimationFrame(this._animFrameId);
    if (this._resizeObserver) this._resizeObserver.disconnect();
  }

  // ========== 绘制逻辑 ==========

  _draw() {
    const ctx = this.ctx;
    const w = this.width;
    const h = this.height;

    ctx.clearRect(0, 0, w, h);

    // 1. 绘制路面背景（浅色暖调）
    this._drawRoadSurface(ctx, w, h);

    // 2. 保存状态，移动到车辆中心视角
    ctx.save();
    ctx.translate(w / 2, h / 2);

    // 3. 绘制路面标记（相对于车辆移动）
    this._drawRoadMarkings(ctx, w, h);

    // 4. 绘制四轮轨迹
    this._drawWheelTrails(ctx);

    // 5. 绘制障碍物
    this._drawObstacles(ctx);

    // 5.5 绘制放置动画反馈
    this._drawPlaceEffects(ctx);

    // 6. 绘制车辆（始终在画布中心）
    this._drawVehicle(ctx);

    // 7. 碰撞时整车闪红
    if (this._collisionFlash > 0.05) {
      ctx.globalAlpha = this._collisionFlash * 0.3;
      ctx.fillStyle = '#d94f4f';
      ctx.fillRect(-w / 2, -h / 2, w, h);
      ctx.globalAlpha = 1;
    }

    ctx.restore();

    // 8. 绘制比例尺
    this._drawScaleBar(ctx, w, h);
  }

  /** 路面背景着色 —— 浅色暖调，根据 mu 值微调 */
  _drawRoadSurface(ctx, w, h) {
    const mu = this.state.mu;
    let color;
    if (mu <= 0.25) {
      // 冰面：浅蓝灰
      color = '#dce8ef';
    } else if (mu <= 0.55) {
      // 湿滑：浅灰
      color = '#e2ddd7';
    } else {
      // 干燥：暖砂石色
      color = '#e8e0d6';
    }

    ctx.fillStyle = color;
    ctx.fillRect(0, 0, w, h);

    // 路面颗粒纹理（微妙的点）
    ctx.globalAlpha = 0.08;
    for (let i = 0; i < 60; i++) {
      const px = ((i * 137.5 + this.state.x * this.scale) % w + w) % w;
      const py = ((i * 97.3 + this.state.y * this.scale) % h + h) % h;
      ctx.fillStyle = mu <= 0.25 ? '#9bb8cc' : '#b0a090';
      ctx.fillRect(px, py, 1.5, 1.5);
    }
    ctx.globalAlpha = 1;

    // 冰面反光效果
    if (mu <= 0.25) {
      const grad = ctx.createRadialGradient(w * 0.3, h * 0.4, 0, w * 0.3, h * 0.4, w * 0.5);
      grad.addColorStop(0, 'rgba(180, 220, 245, 0.15)');
      grad.addColorStop(1, 'transparent');
      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, w, h);
    }
  }

  /** 路面标记线（网格参考线） */
  _drawRoadMarkings(ctx, w, h) {
    const s = this.scale;
    const spacing = this._roadMarkSpacing * s;
    const offsetX = (-this.state.x * s) % spacing;
    const offsetY = (-this.state.y * s) % spacing;

    ctx.strokeStyle = 'rgba(160, 150, 140, 0.2)';
    ctx.lineWidth = 0.5;
    ctx.setLineDash([8, 6]);

    // 水平线
    for (let y = offsetY - spacing; y < h; y += spacing) {
      ctx.beginPath();
      ctx.moveTo(-w / 2, y);
      ctx.lineTo(w / 2, y);
      ctx.stroke();
    }

    // 垂直线
    for (let x = offsetX - spacing; x < w; x += spacing) {
      ctx.beginPath();
      ctx.moveTo(x, -h / 2);
      ctx.lineTo(x, h / 2);
      ctx.stroke();
    }

    ctx.setLineDash([]);
  }

  /** 绘制四轮轨迹线 */
  _drawWheelTrails(ctx) {
    const s = this.scale;
    const cx = this.state.x;
    const cy = this.state.y;

    // ESP OFF 时降低透明度，ESP ON 时正常
    const globalAlpha = this.state.espEnabled ? 0.8 : 0.4;
    const baseLineWidth = this.state.espEnabled ? 1.8 : 1.2;

    for (const [wheelName, trail] of Object.entries(this.trails)) {
      if (trail.length < 2) continue;
      const color = this.trailColors[wheelName];

      for (let i = 1; i < trail.length; i++) {
        const alpha = (i / trail.length) * globalAlpha;
        const lineWidth = baseLineWidth * (0.5 + 0.5 * (i / trail.length));

        ctx.beginPath();
        ctx.strokeStyle = `rgba(${color.r}, ${color.g}, ${color.b}, ${alpha})`;
        ctx.lineWidth = lineWidth;
        ctx.lineCap = 'round';

        const p0 = trail[i - 1];
        const p1 = trail[i];

        // 世界坐标转画布坐标（以车辆为中心的相对坐标）
        const x0 = (p0.x - cx) * s;
        const y0 = -(p0.y - cy) * s;
        const x1 = (p1.x - cx) * s;
        const y1 = -(p1.y - cy) * s;

        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
      }
    }
  }

  /** 绘制障碍物 */
  _drawObstacles(ctx) {
    const s = this.scale;
    const cx = this.state.x;
    const cy = this.state.y;

    for (const obs of this.obstacles) {
      // 世界坐标转画布坐标
      const sx = (obs.x - cx) * s;
      const sy = -(obs.y - cy) * s;
      const sr = obs.radius * s;

      // 判断是否为当前碰撞的障碍物
      const isHit = this.collision.hit && this.collision.obstacle_id === obs.id;

      ctx.save();
      ctx.beginPath();
      ctx.arc(sx, sy, sr, 0, Math.PI * 2);

      if (isHit && this._collisionFlash > 0.05) {
        // 碰撞闪烁：红色脉冲
        const flashAlpha = 0.5 + 0.5 * Math.sin(this._flashPhase * 12);
        ctx.fillStyle = `rgba(217, 79, 79, ${flashAlpha})`;
        ctx.strokeStyle = '#a03030';
        ctx.shadowColor = 'rgba(217, 79, 79, 0.6)';
        ctx.shadowBlur = 10;
      } else {
        // 正常状态：深灰/棕色
        ctx.fillStyle = '#6b5b4f';
        ctx.strokeStyle = '#4a3f35';
      }

      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();
      ctx.restore();

      // 障碍物中心十字标记
      ctx.strokeStyle = isHit ? 'rgba(255,255,255,0.7)' : 'rgba(255,255,255,0.3)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(sx - sr * 0.4, sy);
      ctx.lineTo(sx + sr * 0.4, sy);
      ctx.moveTo(sx, sy - sr * 0.4);
      ctx.lineTo(sx, sy + sr * 0.4);
      ctx.stroke();
    }
  }

  /** 绘制障碍物放置动画（扩散圆环） */
  _drawPlaceEffects(ctx) {
    const now = performance.now();
    const duration = 500; // 动画持续 500ms
    const s = this.scale;
    const cx = this.state.x;
    const cy = this.state.y;

    this._placeEffects = this._placeEffects.filter(fx => {
      const elapsed = now - fx.startTime;
      if (elapsed > duration) return false; // 动画结束，移除

      const progress = elapsed / duration;
      const radius = (10 + progress * 30); // 像素半径从10扩散到40
      const alpha = 1 - progress;

      const sx = (fx.x - cx) * s;
      const sy = -(fx.y - cy) * s;

      ctx.save();
      ctx.beginPath();
      ctx.arc(sx, sy, radius, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(224, 122, 63, ${alpha})`;
      ctx.lineWidth = 2.5 * (1 - progress);
      ctx.stroke();

      // 内层小圆点
      ctx.beginPath();
      ctx.arc(sx, sy, 3 * (1 - progress), 0, Math.PI * 2);
      ctx.fillStyle = `rgba(224, 122, 63, ${alpha * 0.8})`;
      ctx.fill();
      ctx.restore();

      return true;
    });
  }

  /** 绘制车辆俯视线框图（浅色主题配色） */
  _drawVehicle(ctx) {
    const psi = this.state.psi;

    ctx.save();
    // 旋转车辆
    ctx.rotate(-psi);

    // 车辆尺寸（像素）
    const bodyW = 60;
    const bodyH = 30;
    const wheelW = 12;
    const wheelH = 6;
    const wheelOffsetX = bodyW * 0.35;
    const wheelOffsetY = bodyH * 0.55;

    // --- 车身 ---
    // 车身阴影（浅色主题用淡阴影）
    ctx.shadowColor = 'rgba(0, 0, 0, 0.15)';
    ctx.shadowBlur = 8;
    ctx.shadowOffsetY = 2;

    ctx.strokeStyle = '#5a4e42';
    ctx.lineWidth = 2;
    ctx.fillStyle = 'rgba(255, 252, 248, 0.92)';

    // 碰撞时车身闪红
    if (this._collisionFlash > 0.05) {
      const flashAlpha = this._collisionFlash * 0.4;
      ctx.fillStyle = `rgba(217, 79, 79, ${flashAlpha})`;
      ctx.strokeStyle = '#d94f4f';
    }

    // 圆角矩形车身
    this._roundRect(ctx, -bodyW / 2, -bodyH / 2, bodyW, bodyH, 5);
    ctx.fill();
    ctx.stroke();

    ctx.shadowColor = 'transparent';
    ctx.shadowBlur = 0;
    ctx.shadowOffsetY = 0;

    // 挡风玻璃线
    ctx.strokeStyle = 'rgba(100, 140, 170, 0.3)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(bodyW * 0.15, -bodyH * 0.38);
    ctx.lineTo(bodyW * 0.15, bodyH * 0.38);
    ctx.stroke();

    // 后窗线
    ctx.beginPath();
    ctx.moveTo(-bodyW * 0.3, -bodyH * 0.35);
    ctx.lineTo(-bodyW * 0.3, bodyH * 0.35);
    ctx.stroke();

    // 车头指示（小三角）— 暖橙色
    ctx.fillStyle = this._collisionFlash > 0.05 ? '#d94f4f' : '#e07a3f';
    ctx.beginPath();
    ctx.moveTo(bodyW / 2 - 2, 0);
    ctx.lineTo(bodyW / 2 - 8, -4);
    ctx.lineTo(bodyW / 2 - 8, 4);
    ctx.closePath();
    ctx.fill();

    // --- 轮胎 ---
    const steeringRad = (this.state.steering / 1080) * (Math.PI / 6); // 最大30度视觉转角

    // 前轮（可转向）
    this._drawWheel(ctx, wheelOffsetX, -wheelOffsetY, wheelW, wheelH, steeringRad, 'FL');
    this._drawWheel(ctx, wheelOffsetX, wheelOffsetY, wheelW, wheelH, steeringRad, 'FR');

    // 后轮（固定）
    this._drawWheel(ctx, -wheelOffsetX, -wheelOffsetY, wheelW, wheelH, 0, 'RL');
    this._drawWheel(ctx, -wheelOffsetX, wheelOffsetY, wheelW, wheelH, 0, 'RR');

    ctx.restore();
  }

  /** 绘制单个车轮 */
  _drawWheel(ctx, x, y, w, h, angle, position) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);

    // 抱死优先级最高：红色闪烁（无 ABS 猛刹的直观反馈）
    const isLocked = this.state.wheelLocked && this.state.wheelLocked[position];
    // 判断是否为 ESP 制动目标轮
    const isTarget = this.state.espActive && this.state.targetWheel === position;
    const flashOn = isTarget && (Math.sin(this._flashPhase * 8) > 0);

    if (isLocked) {
      // 抱死：红色常亮 + 脉冲发光
      const pulse = 0.5 + 0.5 * Math.sin(this._flashPhase * 10);
      ctx.fillStyle = '#d94f4f';
      ctx.strokeStyle = '#a03030';
      ctx.shadowColor = `rgba(217, 79, 79, ${0.4 + 0.4 * pulse})`;
      ctx.shadowBlur = 8;
    } else if (flashOn) {
      // ESP 介入闪烁：暖橙色
      ctx.fillStyle = '#e07a3f';
      ctx.strokeStyle = '#c9603a';
      ctx.shadowColor = 'rgba(224, 122, 63, 0.6)';
      ctx.shadowBlur = 10;
    } else {
      // 正常：深灰轮胎
      ctx.fillStyle = '#3d3d3d';
      ctx.strokeStyle = '#5a5a5a';
      ctx.shadowColor = 'transparent';
      ctx.shadowBlur = 0;
    }

    ctx.lineWidth = 1.5;
    ctx.fillRect(-w / 2, -h / 2, w, h);
    ctx.strokeRect(-w / 2, -h / 2, w, h);

    // 轮胎花纹线
    ctx.strokeStyle = (flashOn || isLocked) ? 'rgba(255,255,255,0.5)' : 'rgba(255,255,255,0.2)';
    ctx.lineWidth = 0.5;
    for (let i = -w / 2 + 3; i < w / 2; i += 3) {
      ctx.beginPath();
      ctx.moveTo(i, -h / 2);
      ctx.lineTo(i, h / 2);
      ctx.stroke();
    }

    ctx.restore();
  }

  /** 绘制比例尺 */
  _drawScaleBar(ctx, w, h) {
    const barMeters = 5;
    const barPixels = barMeters * this.scale;
    const x = w - barPixels - 20;
    const y = h - 20;

    ctx.strokeStyle = 'rgba(90, 78, 66, 0.5)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + barPixels, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 4);
    ctx.moveTo(x + barPixels, y - 4);
    ctx.lineTo(x + barPixels, y + 4);
    ctx.stroke();

    ctx.fillStyle = 'rgba(90, 78, 66, 0.6)';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${barMeters}m`, x + barPixels / 2, y - 8);
  }

  /** 辅助：圆角矩形路径 */
  _roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + w - r, y);
    ctx.quadraticCurveTo(x + w, y, x + w, y + r);
    ctx.lineTo(x + w, y + h - r);
    ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
    ctx.lineTo(x + r, y + h);
    ctx.quadraticCurveTo(x, y + h, x, y + h - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }
}
