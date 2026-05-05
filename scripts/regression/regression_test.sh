#!/usr/bin/env bash
# regression_test.sh
#
# 목적: sonar_3d_reconstruction Phase B 회귀 테스트 orchestrator.
#       동일 bag 을 두 빌드(baseline / candidate) 환경에서 재생하면서
#       /pkrc/sonar/cpp_pointcloud 토픽을 record 하여 알고리즘 동작 보존을
#       검증한다. fast_lio + sonar_3d 두 launch 를 동시에 띄운다.
#
# 사용:
#   bash regression_test.sh baseline    # baseline 측정 (사전 main HEAD 빌드 가정)
#   bash regression_test.sh candidate   # candidate 측정 (현재 branch 빌드 가정)
#   bash regression_test.sh compare     # baseline vs candidate 비교 → regression_compare.py
#
# Phase B-1.0 도입 (refactor/phase-b1-perf-surgical).
#
# 데이터 안전 정책: bag 데이터는 read-only 로만 다룬다. 입력 BAG_PATH 는
# 절대 수정·삭제하지 않으며 record 산출물은 OUT_DIR (기본 /tmp 하위) 에만 쓴다.

set -euo pipefail

# ---------------------------------------------------------------------------
# 환경 변수 (기본값, override 가능)
# ---------------------------------------------------------------------------
: "${BAG_PATH:=/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m3000d_blueboat/m3000d-range15-tilt90}"
: "${PLAY_DURATION:=90}"
: "${OUT_DIR:=/tmp/sonar3d_regression}"
: "${LAUNCH_PKG:=sonar_3d_reconstruction}"
: "${LAUNCH_FILE:=3d_mapping.launch.py}"
: "${SLAM_LAUNCH_PKG:=fast_lio}"
: "${SLAM_LAUNCH_FILE:=mapping.launch.py}"
: "${PC_TOPIC:=/sonar_3d_mapper/point_cloud}"
: "${JACCARD_THRESHOLD:=1.0}"
: "${MEAN_LOG_ODDS_THRESHOLD:=0.0}"

# Dataset profile (P-1/P-2 default = P-2 m3000d/tilt90, fast_lio odom).
# bag 마다 sonar_model/sonar_pitch/odometry 가 다르므로 호출 측에서 override.
# QoS_RELIABILITY 는 bag 의 publisher QoS 와 일치해야 input drop 0 보장 (UCRC bag = best_effort).
: "${SONAR_MODEL:=m3000d}"
: "${SONAR_PITCH:=90.0}"
: "${ODOMETRY:=fast_lio}"
: "${QOS_RELIABILITY:=best_effort}"

# DDS 격리: 같은 컨테이너의 다른 세션 노드와 cross-talk 방지.
# 0 이 아닌 값이면 default group 과 분리됨. override 가능.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# 환경 fail-safe: 호출 셸이 ROS env 를 source 하지 않은 채 들어와도
# launch 가 패키지를 찾도록 자체적으로 source 한다. 이미 source 된
# 호출자는 동일 path 가 prefix 되어도 동작에 영향 없음.
: "${ROS_DISTRO:=humble}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
: "${WORKSPACE_SETUP:=/workspace/ros2_ws/install/setup.bash}"
if [[ -f "${WORKSPACE_SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${WORKSPACE_SETUP}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<EOF
Usage: $(basename "$0") <baseline|candidate|compare>

Modes:
  baseline   replay BAG_PATH on baseline build, record ${PC_TOPIC}
  candidate  replay BAG_PATH on candidate build, record ${PC_TOPIC}
  compare    invoke regression_compare.py against baseline+candidate clouds

Typical flow:
  1) baseline 빌드 (main HEAD 또는 archive baseline) 후
     bash $(basename "$0") baseline
  2) candidate 빌드 (현재 branch) 후
     bash $(basename "$0") candidate
  3) bash $(basename "$0") compare      # 임계 PASS/FAIL 판정
  4) plot/시각 비교 (별도 스크립트)

Environment overrides:
  BAG_PATH                  (default: ${BAG_PATH})
  PLAY_DURATION             (default: ${PLAY_DURATION} sec)
  OUT_DIR                   (default: ${OUT_DIR})
  LAUNCH_PKG/LAUNCH_FILE    (default: ${LAUNCH_PKG} / ${LAUNCH_FILE})
  SLAM_LAUNCH_PKG/FILE      (default: ${SLAM_LAUNCH_PKG} / ${SLAM_LAUNCH_FILE})
  PC_TOPIC                  (default: ${PC_TOPIC})
  JACCARD_THRESHOLD         (default: ${JACCARD_THRESHOLD})
  MEAN_LOG_ODDS_THRESHOLD   (default: ${MEAN_LOG_ODDS_THRESHOLD})
  SONAR_MODEL               (default: ${SONAR_MODEL})    # m750d | m3000d
  SONAR_PITCH               (default: ${SONAR_PITCH})    # 30.0 | 60.0 | 90.0
  ODOMETRY                  (default: ${ODOMETRY})    # cartographer | fast_lio | fast_lio_loc
  QOS_RELIABILITY           (default: ${QOS_RELIABILITY})    # reliable | best_effort
  ROS_DOMAIN_ID             (default: ${ROS_DOMAIN_ID})    # DDS 도메인 격리
EOF
}

# ---------------------------------------------------------------------------
# run_replay <label>
#   label ∈ {baseline, candidate}
# ---------------------------------------------------------------------------
run_replay() {
    local label="$1"
    local out="${OUT_DIR}/${label}"

    if [[ ! -d "${BAG_PATH}" && ! -f "${BAG_PATH}" ]]; then
        echo "[regression_test] ERROR: BAG_PATH not found: ${BAG_PATH}" >&2
        exit 2
    fi

    echo "[regression_test] mode=${label} bag=${BAG_PATH} duration=${PLAY_DURATION}s out=${out}"
    echo "[regression_test] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "[regression_test] profile: sonar=${SONAR_MODEL} pitch=${SONAR_PITCH} odom=${ODOMETRY} qos=${QOS_RELIABILITY}"

    # 1) out 디렉토리 정리 + 생성
    rm -rf "${out}"
    mkdir -p "${out}"

    # 2) fast_lio mapping.launch.py 백그라운드 launch (setsid → 독립 process group)
    #    qos_reliability 는 bag publisher QoS 와 일치해야 fast_lio 가 livox/imu 를 받음.
    echo "[regression_test] launching SLAM: ${SLAM_LAUNCH_PKG} ${SLAM_LAUNCH_FILE}"
    setsid ros2 launch "${SLAM_LAUNCH_PKG}" "${SLAM_LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false foxglove:=false \
        qos_reliability:="${QOS_RELIABILITY}" \
        > "${out}/slam_launch.log" 2>&1 &
    local slam_pgid=$!
    sleep 5

    # 3) sonar_3d mapping.launch.py 백그라운드 launch (setsid → 독립 process group)
    #    sonar_model/sonar_pitch/odometry 는 bag 마다 다르므로 dataset profile 변수에서 forward.
    echo "[regression_test] launching SONAR: ${LAUNCH_PKG} ${LAUNCH_FILE}"
    setsid ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false \
        sonar_model:="${SONAR_MODEL}" \
        sonar_pitch:="${SONAR_PITCH}" \
        odometry:="${ODOMETRY}" \
        qos_reliability:="${QOS_RELIABILITY}" \
        show_opencv:=false \
        launch_visualizer:=false \
        > "${out}/sonar_launch.log" 2>&1 &
    local sonar_pgid=$!
    sleep 8

    # 4) bag record (sqlite3) 백그라운드
    echo "[regression_test] recording ${PC_TOPIC} → ${out}/cloud"
    ros2 bag record -s sqlite3 -o "${out}/cloud" "${PC_TOPIC}" \
        > "${out}/record.log" 2>&1 &
    local record_pid=$!
    sleep 1

    # 5) bag play (timeout 으로 안전하게 종료)
    echo "[regression_test] playing bag for ${PLAY_DURATION}s (timeout +10s safety)"
    timeout "$((PLAY_DURATION + 10))" ros2 bag play "${BAG_PATH}" --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 || true

    # 6) drain + kill (record → sonar → slam 순, process group 단위 3단계 escalation)
    echo "[regression_test] draining 5s"
    sleep 5

    echo "[regression_test] stopping record (pid=${record_pid})"
    kill -INT "${record_pid}" 2>/dev/null || true
    wait "${record_pid}" 2>/dev/null || true

    echo "[regression_test] stopping sonar (pgid=${sonar_pgid})"
    _kill_pgroup "${sonar_pgid}"

    echo "[regression_test] stopping slam (pgid=${slam_pgid})"
    _kill_pgroup "${slam_pgid}"

    echo "[regression_test] mode=${label} done. logs+cloud bag at: ${out}"
}

# ---------------------------------------------------------------------------
# _kill_pgroup <pgid>
#   ros2 launch 의 자식 C++ 노드(예: fastlio_mapping)가 SIGINT 만으로는
#   정리되지 않는 사례가 있어 INT → TERM → KILL 3 단계 escalation.
# ---------------------------------------------------------------------------
_kill_pgroup() {
    local pgid="${1:-}"
    if [[ -z "${pgid}" ]]; then
        return 0
    fi
    kill -INT  "-${pgid}" 2>/dev/null || true
    sleep 5
    kill -TERM "-${pgid}" 2>/dev/null || true
    sleep 5
    kill -KILL "-${pgid}" 2>/dev/null || true
    wait "${pgid}" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# compare_results
# ---------------------------------------------------------------------------
compare_results() {
    local baseline_cloud="${OUT_DIR}/baseline/cloud"
    local candidate_cloud="${OUT_DIR}/candidate/cloud"

    if [[ ! -d "${baseline_cloud}" ]]; then
        echo "[regression_test] ERROR: baseline cloud not found: ${baseline_cloud}" >&2
        echo "[regression_test] hint: run '$(basename "$0") baseline' first" >&2
        exit 3
    fi
    if [[ ! -d "${candidate_cloud}" ]]; then
        echo "[regression_test] ERROR: candidate cloud not found: ${candidate_cloud}" >&2
        echo "[regression_test] hint: run '$(basename "$0") candidate' first" >&2
        exit 3
    fi

    echo "[regression_test] comparing baseline vs candidate"
    python3 "${SCRIPT_DIR}/regression_compare.py" \
        --baseline "${baseline_cloud}" \
        --candidate "${candidate_cloud}" \
        --topic "${PC_TOPIC}" \
        --jaccard-threshold "${JACCARD_THRESHOLD}" \
        --mean-log-odds-threshold "${MEAN_LOG_ODDS_THRESHOLD}"
}

# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------
if [[ $# -lt 1 || -z "${1:-}" ]]; then
    usage
    exit 1
fi

mode="$1"
case "${mode}" in
    baseline)
        run_replay baseline
        ;;
    candidate)
        run_replay candidate
        ;;
    compare)
        compare_results
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        echo "[regression_test] unknown mode: ${mode}" >&2
        usage
        exit 1
        ;;
esac
