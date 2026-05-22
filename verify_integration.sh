#!/bin/bash
#
# Verification Script: ArduPilot SITL + AirSim + OpenVINS Integration
#
# Checks all components are running and data is flowing correctly.
# Run after all systems are started.
#
# Usage: bash /mnt/rnd/Shubham/openvins_ws/integration/verify_integration.sh
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS=0
FAIL=0
WARN=0

pass() { echo -e "${GREEN}[PASS]${NC} $1"; ((PASS++)); }
fail() { echo -e "${RED}[FAIL]${NC} $1"; ((FAIL++)); }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; ((WARN++)); }

echo "============================================"
echo "  Integration Verification Script"
echo "  $(date)"
echo "============================================"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null
source /mnt/rnd/Shubham/openvins_ws/install/setup.bash 2>/dev/null

# ============================================
# Check 1: ROS2 Environment
# ============================================
echo "--- Check 1: ROS2 Environment ---"

if command -v ros2 &>/dev/null; then
    pass "ROS2 CLI available"
else
    fail "ROS2 CLI not found (source /opt/ros/humble/setup.bash)"
fi

if ros2 pkg list 2>/dev/null | grep -q ov_msckf; then
    pass "ov_msckf package found"
else
    fail "ov_msckf package not found (source workspace setup.bash)"
fi

echo ""

# ============================================
# Check 2: AirSim Connection
# ============================================
echo "--- Check 2: AirSim Connection ---"

AIRSIM_CHECK=$(python3 -c "
import airsim
try:
    c = airsim.MultirotorClient()
    c.confirmConnection()
    print('OK')
except Exception as e:
    print(f'FAIL:{e}')
" 2>&1)

if [[ "$AIRSIM_CHECK" == "OK" ]]; then
    pass "AirSim API connection successful"
else
    fail "AirSim not reachable: $AIRSIM_CHECK"
fi

# Check IMU data
IMU_CHECK=$(python3 -c "
import airsim
try:
    c = airsim.MultirotorClient()
    c.confirmConnection()
    imu = c.getImuData()
    if imu.time_stamp > 0:
        print(f'OK:ts={imu.time_stamp},az={imu.linear_acceleration.z_val:.2f}')
    else:
        print('FAIL:zero_timestamp')
except Exception as e:
    print(f'FAIL:{e}')
" 2>&1)

if [[ "$IMU_CHECK" == OK* ]]; then
    pass "AirSim IMU data: ${IMU_CHECK#OK:}"
else
    fail "AirSim IMU data: $IMU_CHECK"
fi

# Check camera
CAM_CHECK=$(python3 -c "
import airsim
try:
    c = airsim.MultirotorClient()
    c.confirmConnection()
    imgs = c.simGetImages([airsim.ImageRequest('front_center', airsim.ImageType.Scene, False, False)])
    if imgs and len(imgs) > 0 and imgs[0].width > 0:
        print(f'OK:{imgs[0].width}x{imgs[0].height}')
    else:
        # Try alternate camera name
        imgs = c.simGetImages([airsim.ImageRequest('front_center_custom', airsim.ImageType.Scene, False, False)])
        if imgs and len(imgs) > 0 and imgs[0].width > 0:
            print(f'OK_ALT:{imgs[0].width}x{imgs[0].height}')
        else:
            print('FAIL:empty_image')
except Exception as e:
    print(f'FAIL:{e}')
" 2>&1)

if [[ "$CAM_CHECK" == OK* ]]; then
    pass "AirSim camera: ${CAM_CHECK#OK:}"
    if [[ "$CAM_CHECK" == OK_ALT* ]]; then
        warn "Camera name is 'front_center_custom' not 'front_center' - update bridge config"
    fi
else
    fail "AirSim camera: $CAM_CHECK"
fi

echo ""

# ============================================
# Check 3: ROS2 Topics
# ============================================
echo "--- Check 3: ROS2 Topics ---"

TOPICS=$(ros2 topic list 2>/dev/null)

# Check IMU topic
if echo "$TOPICS" | grep -q "^/imu$"; then
    pass "/imu topic exists"

    # Check rate
    IMU_HZ=$(timeout 5 ros2 topic hz /imu --window 20 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
    if [[ -n "$IMU_HZ" ]]; then
        IMU_HZ_INT=${IMU_HZ%.*}
        if (( IMU_HZ_INT > 150 )); then
            pass "/imu rate: ${IMU_HZ} Hz"
        elif (( IMU_HZ_INT > 50 )); then
            warn "/imu rate: ${IMU_HZ} Hz (expected ~200 Hz)"
        else
            fail "/imu rate: ${IMU_HZ} Hz (too low, expected ~200 Hz)"
        fi
    else
        warn "/imu rate: could not measure (topic may be intermittent)"
    fi
else
    fail "/imu topic not found"
fi

# Check camera topic
if echo "$TOPICS" | grep -q "^/camera/image$"; then
    pass "/camera/image topic exists"

    CAM_HZ=$(timeout 8 ros2 topic hz /camera/image --window 10 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
    if [[ -n "$CAM_HZ" ]]; then
        CAM_HZ_INT=${CAM_HZ%.*}
        if (( CAM_HZ_INT > 15 )); then
            pass "/camera/image rate: ${CAM_HZ} Hz"
        elif (( CAM_HZ_INT > 5 )); then
            warn "/camera/image rate: ${CAM_HZ} Hz (expected ~30 Hz)"
        else
            fail "/camera/image rate: ${CAM_HZ} Hz (too low)"
        fi
    else
        warn "/camera/image rate: could not measure"
    fi
else
    fail "/camera/image topic not found"
fi

# Check camera_info
if echo "$TOPICS" | grep -q "^/camera/camera_info$"; then
    pass "/camera/camera_info topic exists"
else
    warn "/camera/camera_info topic not found (optional for OpenVINS)"
fi

echo ""

# ============================================
# Check 4: OpenVINS Status
# ============================================
echo "--- Check 4: OpenVINS Status ---"

# Check if OpenVINS node is running
if ros2 node list 2>/dev/null | grep -q "ov_msckf"; then
    pass "OpenVINS node running"
else
    warn "OpenVINS node not detected (may not be started yet)"
fi

# Check OpenVINS output topics
if echo "$TOPICS" | grep -q "/ov_msckf/odomimu"; then
    pass "/ov_msckf/odomimu topic exists (VIO outputting poses)"

    VIO_HZ=$(timeout 8 ros2 topic hz /ov_msckf/odomimu --window 10 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
    if [[ -n "$VIO_HZ" ]]; then
        pass "VIO output rate: ${VIO_HZ} Hz"
    else
        warn "VIO output rate: could not measure (may not be initialized)"
    fi
else
    warn "/ov_msckf/odomimu not found (OpenVINS may not be initialized - need motion)"
fi

if echo "$TOPICS" | grep -q "/ov_msckf/pathimu"; then
    pass "/ov_msckf/pathimu topic exists"
fi

if echo "$TOPICS" | grep -q "/ov_msckf/points_msckf"; then
    pass "/ov_msckf/points_msckf topic exists"
fi

echo ""

# ============================================
# Check 5: Timestamp Consistency
# ============================================
echo "--- Check 5: Timestamp Consistency ---"

TS_CHECK=$(python3 -c "
import subprocess, json, re

# Get one IMU message
result = subprocess.run(['ros2', 'topic', 'echo', '/imu', '--once', '--no-daemon'],
                       capture_output=True, text=True, timeout=10)
imu_out = result.stdout

# Extract timestamp
sec_match = re.search(r'sec:\s*(\d+)', imu_out)
nsec_match = re.search(r'nanosec:\s*(\d+)', imu_out)

if sec_match and nsec_match:
    sec = int(sec_match.group(1))
    nsec = int(nsec_match.group(1))
    if sec > 0:
        print(f'OK:sec={sec},nsec={nsec}')
    else:
        print(f'WARN:sec=0,nsec={nsec}')
else:
    print('FAIL:no_timestamp')
" 2>&1)

if [[ "$TS_CHECK" == OK* ]]; then
    pass "IMU timestamps valid: ${TS_CHECK#OK:}"
elif [[ "$TS_CHECK" == WARN* ]]; then
    warn "IMU timestamps: ${TS_CHECK#WARN:}"
else
    warn "Timestamp check: $TS_CHECK"
fi

echo ""

# ============================================
# Check 6: Config Files
# ============================================
echo "--- Check 6: Configuration Files ---"

CONFIG_DIR="/mnt/rnd/Shubham/openvins_ws/integration/openvins_airsim_config"

if [[ -f "$CONFIG_DIR/estimator_config.yaml" ]]; then
    pass "estimator_config.yaml exists"
else
    fail "estimator_config.yaml missing"
fi

if [[ -f "$CONFIG_DIR/kalibr_imu_chain.yaml" ]]; then
    pass "kalibr_imu_chain.yaml exists"
else
    fail "kalibr_imu_chain.yaml missing"
fi

if [[ -f "$CONFIG_DIR/kalibr_imucam_chain.yaml" ]]; then
    pass "kalibr_imucam_chain.yaml exists"
else
    fail "kalibr_imucam_chain.yaml missing"
fi

if [[ -f "/mnt/rnd/Shubham/openvins_ws/integration/airsim_openvins_bridge.py" ]]; then
    pass "airsim_openvins_bridge.py exists"
else
    fail "airsim_openvins_bridge.py missing"
fi

echo ""

# ============================================
# Summary
# ============================================
echo "============================================"
echo "  Summary"
echo "============================================"
echo -e "  ${GREEN}PASS: $PASS${NC}"
echo -e "  ${RED}FAIL: $FAIL${NC}"
echo -e "  ${YELLOW}WARN: $WARN${NC}"
echo ""

if (( FAIL == 0 )); then
    echo -e "${GREEN}All critical checks passed!${NC}"
    exit 0
else
    echo -e "${RED}$FAIL critical check(s) failed. See above for details.${NC}"
    exit 1
fi
