#!/usr/bin/env bash
set -euo pipefail

USER_HOME="/home/user1"
MONITORING_DIR="$USER_HOME/UC3/monitoring"
SESSION="monitor_other"

echo "Stopping monitoring session: ${SESSION}"

# Kill the tmux session if it exists
if tmux ls 2>/dev/null | grep -q "^${SESSION}:"; then
  echo "Killing tmux session ${SESSION}..."
  tmux kill-session -t "${SESSION}"
else
  echo "No tmux session named ${SESSION} found."
fi

# Give some time for children to exit
sleep 1

echo "Cleaning leftover oracle.py processes (if any)..."
# Find oracle.py processes that look like monitoring instances and terminate them gracefully
mapfile -t ORACLE_PIDS < <(pgrep -f "oracle.py" || true)
for pid in "${ORACLE_PIDS[@]:-}"; do
  if [ -n "$pid" ]; then
    cmd=$(ps -p "$pid" -o args= --no-headers || true)
    if echo "$cmd" | grep -q "--property" || echo "$cmd" | grep -q "TLOracle"; then
      echo "Terminating oracle.py pid=$pid ($cmd)"
      kill "$pid" || true
    fi
  fi
done

echo "Cleaning leftover ros2 launch / ros2 nodes started by monitor (if any)..."
# Kill ros2 launch commands that run the monitor launch file
mapfile -t ROS_PIDS < <(pgrep -f "ros2 launch monitor/launch/monitor.launch" || true)
for pid in "${ROS_PIDS[@]:-}"; do
  if [ -n "$pid" ]; then
    cmd=$(ps -p "$pid" -o args= --no-headers || true)
    echo "Terminating ros2 launcher pid=$pid ($cmd)"
    kill "$pid" || true
  fi
done

# Final safety: if any of the above did not exit, escalate after a short wait
sleep 1
for p in "${ORACLE_PIDS[@]:-}" "${ROS_PIDS[@]:-}"; do
  if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then
    echo "Process $p still running, sending SIGKILL"
    kill -9 "$p" || true
  fi
done

echo "Stop script completed."
