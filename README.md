# ESP 车身电子稳定系统 · 教学模拟程序

一个可在浏览器里跑的线控底盘教学仿真：单车模型 + 车轮旋转动力学 + 六代底盘技术演进（裸车 → ABS → EBD → ASR → ESP → 扭矩矢量），支持障碍物布局、手动碰撞测试和实时数据看板。

后端 Flask + SocketIO（100Hz 仿真计算 / 20Hz 前端推送），前端原生 JS + Canvas，无构建步骤。

## 快速开始

### 方式一：Windows 一键启动

双击 `启动仿真.bat`，自动安装依赖并打开浏览器。

### 方式二：手动运行

```bash
# 需要 Python 3.9+
pip install -r requirements.txt
python sim/app.py
```

然后浏览器访问 `http://localhost:5000`。

## 能玩什么

- **六代技术切换**：顶部时代选择器覆盖 裸车 / ABS / EBD / ASR / ESP / 扭矩矢量，同一脚刹车在不同时代是抱死失控还是稳稳刹停，一比就知道
- **真实车轮动力学**：μ-κ 滑移曲线、摩擦圆耦合、轮速滑移率逐轮计算，抱死车轮在视图里红色脉冲
- **5 种障碍物布局**：随机散布 / 密集区域 / 走廊通道 / 绕桩 / 冰面随机，支持手动放置和删除障碍物解除碰撞
- **多路面工况**：干地 / 湿地 / 冰面摩擦系数一键切换
- **实时图表**：横摆角速度 vs 参考值、质心侧偏角、轮速、ESP 各轮制动压力介入状态

## 目录结构

```
sim/
  app.py              Flask 服务 + SimulationEngine（碰撞检测、WebSocket 推送）
  vehicle_model.py    单车模型与车辆动力学
  chassis_systems.py  车轮旋转动力学 + 六代底盘系统（ERAS 注册表）
  esp_controller.py   ESP 横摆闭环差动制动控制器
  scenarios.py        障碍物布局方案定义
static/
  index.html          页面入口
  css/  js/           样式、Canvas 视图、图表与控制逻辑
requirements.txt
启动仿真.bat          Windows 一键启动脚本
```

## API 速览

| 接口 | 说明 |
| --- | --- |
| `GET /api/eras` / `POST /api/era` | 查询/切换技术时代 |
| `POST /api/scenario` | 加载障碍物布局方案 |
| `POST /api/reset` | 重新生成布局并回原点 |
| `POST /api/reset_position` | 仅车辆回原点（布局不变） |
| `POST /api/obstacle/delete` | 删除指定障碍物 |

WebSocket 事件 `sim_data` 推送完整的车辆状态、轮端数据、障碍物与碰撞信息。

## 自测

```bash
python -m sim.chassis_systems
```

输出干/冰路面制动距离与转向保持基准（如：100km/h 干地刹距 裸车 50.1m / ABS 49.5m）。

## 许可

[MIT](LICENSE)
