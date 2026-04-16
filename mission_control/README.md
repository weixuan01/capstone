# Mission Control

A browser-based GUI for launching and monitoring Crazyflie drone missions.

---

## One-time setup

### 1. Python dependencies

    pip install fastapi uvicorn websockets ptyprocess --break-system-packages

### 2. ROS 2 optional packages (for battery data)

    sudo apt install ros-humble-rosbridge-suite

rosbridge provides live battery data via WebSocket on ws://localhost:9090.
Run manually or add to your shared launch file:

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

If rosbridge is not running, battery bars show — and recall/land buttons are disabled.
Everything else still works.

---

## Running Mission Control

    cd ~/capstone_beta/mission_control
    ./start.sh

If not executable:

    chmod +x start.sh
    ./start.sh

The start.sh script starts the Mission Control web server on port 8000 and opens
http://localhost:8000 in your browser.

---

## Using Mission Control

### Launch Config tab

1. Toggle Sim or Real mode
2. In Sim mode, select a world (maze or circle-maze) or choose Custom path
   In Real mode, type an optional map path
3. Add drones with + Add Drone
   For each drone set: prefix (e.g. cf1), mission mode, and radio address
4. Click Launch — runs launch-universal.sh and switches to the Monitor tab

### Monitor tab

Left panel — Drone Status:
- Battery voltage bar per drone (requires rosbridge)
- Land button per drone — publishes a descending cmd_vel
- Land All Drones button — lands all drones simultaneously

Right panel — Launch Log:
- Full terminal output streamed from launch-universal.sh
- Stop Launch button sends SIGINT to the launch process group (clean ROS 2 shutdown)

---

## File overview

    start.sh      — starts the Mission Control web server
    server.py     — FastAPI backend (WebSocket terminal + launch runner)
    index.html    — Mission Control frontend

---

## Troubleshooting

Permission denied:

    chmod +x start.sh

Port 8000 already in use:

    pkill -f uvicorn
    ./start.sh

rosbridge not connecting — run manually:

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
