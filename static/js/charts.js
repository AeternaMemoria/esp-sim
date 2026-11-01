/**
 * ChartsManager — ESP 教学模拟：实时曲线图表管理
 * 使用 Chart.js 4.x，4个图表：横摆角速度、侧向加速度、四轮滑移率、车速
 */
class ChartsManager {
  constructor(containerElement) {
    this.container = containerElement;
    this.maxPoints = 200; // 10秒 @20Hz
    this.charts = {};
    this.timeLabels = [];

    // 创建 DOM 结构
    this._buildDOM();
    // 初始化图表
    this._initCharts();
  }

  /** 构建图表容器 DOM */
  _buildDOM() {
    this.container.innerHTML = `
      <div class="chart-card" id="chart-yaw">
        <div class="chart-card__title">横摆角速度 (deg/s)</div>
        <div class="chart-card__canvas-wrap"><canvas id="canvas-yaw"></canvas></div>
      </div>
      <div class="chart-card" id="chart-lat">
        <div class="chart-card__title">侧向加速度 (m/s²)</div>
        <div class="chart-card__canvas-wrap"><canvas id="canvas-lat"></canvas></div>
      </div>
      <div class="chart-card" id="chart-slip">
        <div class="chart-card__title">四轮滑移率 (%)</div>
        <div class="chart-card__canvas-wrap"><canvas id="canvas-slip"></canvas></div>
      </div>
      <div class="chart-card" id="chart-speed">
        <div class="chart-card__title">车速 (km/h)</div>
        <div class="chart-card__canvas-wrap"><canvas id="canvas-speed"></canvas></div>
      </div>
    `;
  }

  /** 通用图表配置（浅色暖调主题） */
  _baseOptions(yLabel, yMin, yMax) {
    return {
      responsive: true,
      maintainAspectRatio: false,
      animation: { duration: 0 },
      interaction: { mode: 'nearest', axis: 'x', intersect: false },
      plugins: {
        legend: {
          display: true,
          position: 'top',
          align: 'end',
          labels: {
            boxWidth: 10,
            boxHeight: 2,
            padding: 8,
            font: { family: 'JetBrains Mono', size: 9 },
            color: '#7a7a7a'
          }
        },
        tooltip: { enabled: false }
      },
      scales: {
        x: {
          display: true,
          grid: { color: 'rgba(224, 214, 204, 0.4)', drawBorder: false },
          ticks: {
            font: { family: 'JetBrains Mono', size: 8 },
            color: '#a09a94',
            maxTicksLimit: 6,
            maxRotation: 0
          },
          border: { display: false }
        },
        y: {
          display: true,
          min: yMin,
          max: yMax,
          grid: { color: 'rgba(224, 214, 204, 0.4)', drawBorder: false },
          ticks: {
            font: { family: 'JetBrains Mono', size: 8 },
            color: '#a09a94',
            maxTicksLimit: 5
          },
          border: { display: false },
          title: {
            display: false
          }
        }
      },
      elements: {
        point: { radius: 0, hitRadius: 0 },
        line: { borderWidth: 1.5, tension: 0.3 }
      }
    };
  }

  /** 初始化所有图表 */
  _initCharts() {
    // 1. 横摆角速度对比图
    const yawCtx = document.getElementById('canvas-yaw').getContext('2d');
    this.charts.yaw = new Chart(yawCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: '实际',
            data: [],
            borderColor: '#4a8fbd',
            backgroundColor: 'rgba(74, 143, 189, 0.06)',
            fill: true
          },
          {
            label: '期望',
            data: [],
            borderColor: '#5a9e6f',
            borderDash: [4, 3],
            backgroundColor: 'transparent',
            fill: false
          }
        ]
      },
      options: this._baseOptions('deg/s', -30, 30)
    });

    // 2. 侧向加速度图
    const latCtx = document.getElementById('canvas-lat').getContext('2d');
    this.charts.lat = new Chart(latCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: '侧向加速度',
            data: [],
            borderColor: '#e07a3f',
            backgroundColor: 'rgba(224, 122, 63, 0.06)',
            fill: true
          },
          {
            label: '+μg 界限',
            data: [],
            borderColor: 'rgba(217, 79, 79, 0.45)',
            borderDash: [6, 4],
            borderWidth: 1,
            backgroundColor: 'transparent',
            fill: false,
            pointRadius: 0
          },
          {
            label: '-μg 界限',
            data: [],
            borderColor: 'rgba(217, 79, 79, 0.45)',
            borderDash: [6, 4],
            borderWidth: 1,
            backgroundColor: 'transparent',
            fill: false,
            pointRadius: 0
          }
        ]
      },
      options: this._baseOptions('m/s²', -12, 12)
    });

    // 3. 四轮滑移率图
    const slipCtx = document.getElementById('canvas-slip').getContext('2d');
    this.charts.slip = new Chart(slipCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'FL', data: [], borderColor: '#e07a3f', backgroundColor: 'transparent' },
          { label: 'FR', data: [], borderColor: '#d94f4f', backgroundColor: 'transparent' },
          { label: 'RL', data: [], borderColor: '#4a8fbd', backgroundColor: 'transparent' },
          { label: 'RR', data: [], borderColor: '#3d8fa8', backgroundColor: 'transparent' }
        ]
      },
      options: this._baseOptions('%', 0, 100)
    });

    // 4. 车速图
    const speedCtx = document.getElementById('canvas-speed').getContext('2d');
    this.charts.speed = new Chart(speedCtx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: '车速',
            data: [],
            borderColor: '#3d3d3d',
            backgroundColor: 'rgba(61, 61, 61, 0.04)',
            fill: true
          }
        ]
      },
      options: this._baseOptions('km/h', 0, 200)
    });
  }

  /**
   * 追加新数据点
   * @param {Object} data — WebSocket sim_data 格式
   */
  update(data) {
    if (!data) return;

    const time = data.time || 0;
    const timeLabel = time.toFixed(1) + 's';

    // 横摆角速度
    const actualYaw = data.sensors?.yaw_rate || data.vehicle?.omega_z || 0;
    const desiredYaw = data.esp?.omega_desired || 0;
    this._pushData(this.charts.yaw, timeLabel, [actualYaw, desiredYaw]);

    // 侧向加速度 + μg界限
    const latAccel = data.sensors?.lateral_accel || 0;
    const mu = data.scenario?.mu || 0.9;
    const muG = mu * 9.81;
    this._pushData(this.charts.lat, timeLabel, [latAccel, muG, -muG]);

    // 四轮滑移率（转为百分比）
    const wheels = data.wheels || {};
    const slipFL = (wheels.fl?.slip || 0) * 100;
    const slipFR = (wheels.fr?.slip || 0) * 100;
    const slipRL = (wheels.rl?.slip || 0) * 100;
    const slipRR = (wheels.rr?.slip || 0) * 100;
    this._pushData(this.charts.slip, timeLabel, [slipFL, slipFR, slipRL, slipRR]);

    // 车速（m/s -> km/h）
    const vx = data.vehicle?.vx || 0;
    const speedKmh = Math.abs(vx) * 3.6;
    this._pushData(this.charts.speed, timeLabel, [speedKmh]);

    // 更新所有图表
    Object.values(this.charts).forEach(chart => chart.update('none'));
  }

  /** 向图表追加数据 */
  _pushData(chart, label, values) {
    chart.data.labels.push(label);
    values.forEach((val, i) => {
      if (chart.data.datasets[i]) {
        chart.data.datasets[i].data.push(val);
      }
    });

    // 超出最大点数时移除旧数据
    if (chart.data.labels.length > this.maxPoints) {
      chart.data.labels.shift();
      chart.data.datasets.forEach(ds => ds.data.shift());
    }
  }

  /** 清空所有图表 */
  reset() {
    Object.values(this.charts).forEach(chart => {
      chart.data.labels = [];
      chart.data.datasets.forEach(ds => { ds.data = []; });
      chart.update('none');
    });
    this.timeLabels = [];
  }

  /** 销毁图表实例 */
  destroy() {
    Object.values(this.charts).forEach(chart => chart.destroy());
    this.charts = {};
  }
}
