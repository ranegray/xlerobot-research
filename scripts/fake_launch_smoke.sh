#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/ros2_ws"
LOG_FILE="$(mktemp -t xle_fake_launch_smoke.XXXXXX.log)"
EVENT_FILE="$(mktemp -t xle_fake_launch_event.XXXXXX.log)"
JOINT_STATE_FILE="$(mktemp -t xle_fake_joint_state.XXXXXX.log)"
LAUNCH_PID=""
EVENT_PID=""

cleanup() {
  if [ -n "${EVENT_PID}" ] && kill -0 "${EVENT_PID}" >/dev/null 2>&1; then
    kill "${EVENT_PID}" >/dev/null 2>&1 || true
    wait "${EVENT_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  rm -f "${LOG_FILE}" "${EVENT_FILE}" "${JOINT_STATE_FILE}"
}
trap cleanup EXIT

wait_for_log() {
  local pattern="$1"
  local timeout_s="$2"
  local start_s="${SECONDS}"
  while true; do
    if grep -q "${pattern}" "${LOG_FILE}"; then
      return 0
    fi
    if [ $((SECONDS - start_s)) -ge "${timeout_s}" ]; then
      echo "timed out waiting for launch log pattern: ${pattern}" >&2
      cat "${LOG_FILE}" >&2
      return 1
    fi
    sleep 0.25
  done
}

if ! command -v timeout >/dev/null 2>&1; then
  echo "fake launch smoke requires GNU timeout; run it in the ROS 2 Linux CI/container environment." >&2
  exit 2
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "${WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
else
  echo "missing ${WS}/install/setup.bash; run 'make build' first." >&2
  exit 2
fi

ros2 launch xle_bringup fake_one_arm.launch.py >"${LOG_FILE}" 2>&1 &
LAUNCH_PID="$!"

sleep 1
if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
  echo "fake launch exited before smoke checks completed." >&2
  cat "${LOG_FILE}" >&2
  exit 1
fi

wait_for_log "joint trajectory guard ready" 20
wait_for_log "fake STS3215 publishing" 20

if ! timeout 10s ros2 topic echo --once /joint_states >"${JOINT_STATE_FILE}" 2>&1; then
  echo "did not observe /joint_states." >&2
  cat "${LOG_FILE}" >&2
  cat "${JOINT_STATE_FILE}" >&2
  exit 1
fi

timeout 10s ros2 topic echo --once /harness/events >"${EVENT_FILE}" 2>&1 &
EVENT_PID="$!"
sleep 1

python3 "${ROOT}/scripts/publish_trajectory_fixture.py" \
  "${ROOT}/tests/fixtures/trajectories/valid_left_arm_small.yaml" \
  >>"${LOG_FILE}" 2>&1

set +e
wait "${EVENT_PID}"
event_status=$?
EVENT_PID=""
set -e

if [ "${event_status}" -ne 0 ]; then
  echo "did not observe /harness/events after publishing fixture." >&2
  cat "${LOG_FILE}" >&2
  cat "${EVENT_FILE}" >&2
  exit 1
fi

if ! grep -q "command_accepted" "${EVENT_FILE}"; then
  echo "guard event did not include command_accepted." >&2
  cat "${EVENT_FILE}" >&2
  exit 1
fi

echo "fake launch smoke passed"
