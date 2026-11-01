@echo off
chcp 65001 >nul
echo ========================================
echo   ESP 车身电子稳定系统 - 教学模拟程序
echo ========================================
echo.
echo 正在检查依赖...
cd /d "%~dp0"
pip install -r requirements.txt -q
echo.
echo 正在启动仿真服务...
echo 浏览器将自动打开 http://localhost:5000
echo 按 Ctrl+C 停止服务
echo.
start http://localhost:5000
python sim/app.py
pause
