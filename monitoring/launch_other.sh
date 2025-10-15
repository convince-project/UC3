#!/usr/bin/env bash
set -euo pipefail

USER_HOME="/home/user1"
MONITORING_DIR="$USER_HOME/UC3/monitoring"
ORACLE_PY="$USER_HOME/ROSMonitoring/oracle/TLOracle/oracle.py"
PY_ENV="$USER_HOME/monitoring-python-env/bin/python"
# Proprietà da 1 a 9 (escluso 4) e da 11 a 12 (salta la 10)
PROPS=(1 2 3 5 6 7 8 9 11)
WS_PATTERN="$MONITORING_DIR/monitor_prop%s_ws/src"
PROPERTY_PREFIX="prop"
BASE_PORT=8080

# crea una sessione tmux
SESSION="monitor_other"
tmux new-session -d -s $SESSION

for prop in "${PROPS[@]}"; do
  WS_DIR=$(printf "$WS_PATTERN" "$prop")
  PROP_NAME="${PROPERTY_PREFIX}${prop}"
  PORT=$((BASE_PORT + prop))

  tmux new-window -t $SESSION -n "PROP${prop}"
  tmux send-keys -t $SESSION:"PROP${prop}".0 "cd ${MONITORING_DIR} && ${PY_ENV} ${ORACLE_PY} --online --dense --property ${PROP_NAME} --port ${PORT}" C-m
  tmux split-window -h -t $SESSION:"PROP${prop}"
  tmux send-keys -t $SESSION:"PROP${prop}".1 "cd ${WS_DIR} && source install/setup.bash && ros2 launch monitor/launch/monitor.launch" C-m

done

tmux select-window -t $SESSION:0
tmux attach -t $SESSION
