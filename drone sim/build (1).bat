@echo off
echo === Building Drone Flight Simulator ===
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m PyInstaller --noconfirm --onefile --windowed --name DroneFlightSimulator --collect-all ursina --collect-all panda3d main.py
echo.
echo Done. Your EXE is at dist\DroneFlightSimulator.exe
pause
