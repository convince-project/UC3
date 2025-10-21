#!/usr/bin/env bash
set -euo pipefail

USER_HOME="/home/user1"
MONITORING_DIR="$USER_HOME/UC3/monitoring"
ORACLE_PY="$USER_HOME/ROSMonitoring/oracle/TLOracle/oracle.py"
PY_ENV="$USER_HOME/monitoring-python-env/bin/python"
POIS=($(seq 1 5))
WS_PATTERN="$MONITORING_DIR/monitor_propPOI%s_ws/src"
PROPERTY_PREFIX="prop10POI"
BASE_PORT=8050

# crea una sessione tmux
SESSION="monitor_prop10POI"
tmux new-session -d -s $SESSION

for poi in "${POIS[@]}"; do
  WS_DIR=$(printf "$WS_PATTERN" "$poi")
  PROP_NAME="${PROPERTY_PREFIX}${poi}"
  PORT=$((BASE_PORT + poi))

  tmux new-window -t $SESSION -n "POI${poi}"
  tmux send-keys -t $SESSION:"POI${poi}".0 "cd ${MONITORING_DIR} && ${PY_ENV} ${ORACLE_PY} --online --dense --property ${PROP_NAME} --port ${PORT}" C-m
  tmux split-window -h -t $SESSION:"POI${poi}"
  tmux send-keys -t $SESSION:"POI${poi}".1 "cd ${WS_DIR} && source install/setup.bash && ros2 launch monitor/launch/monitor.launch" C-m

done

tmux select-window -t $SESSION:0
tmux attach -t $SESSION
