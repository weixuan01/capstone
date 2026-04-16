#!/bin/bash
# ─────────────────────────────────────────────
# start.sh — starts the Mission Control server
# ─────────────────────────────────────────────

SCRIPT_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")")

pkill -f uvicorn 2>/dev/null
sleep 1

echo "Starting Mission Control server..."
cd "$SCRIPT_DIR"
uvicorn server:app --host 127.0.0.1 --port 8000 &
SERVER_PID=$!
sleep 1

xdg-open http://localhost:8000 2>/dev/null &

echo ""
echo "──────────────────────────────────────────"
echo "  Mission Control: http://localhost:8000"
echo "  Press Ctrl+C to stop."
echo "──────────────────────────────────────────"

trap "echo '[INFO] Shutting down...'; kill $SERVER_PID 2>/dev/null" EXIT
wait $SERVER_PID
