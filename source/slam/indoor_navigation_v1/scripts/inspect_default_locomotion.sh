#!/bin/bash
# Read-only, run with sudo solely to inspect the existing xxx deployment.
set -eu
task_sdk=/home/xxx/goai_embodied_future_material/src/S10_sdk_deploy
printf '\n--- Startup (no general environment dump) ---\n'
sed -n '1,180p' /home/xxx/tremor_test/start_rl.sh
printf '\n--- Locomotion-only environment allowlist ---\n'
grep -nE '^(export )?(S10_|ROS_DOMAIN_ID=|RMW_IMPLEMENTATION=|FASTRTPS_DEFAULT_PROFILES_FILE=)' /home/xxx/tremor_test/rl_env.sh || true
printf '\n--- Current entry point ---\n'
sed -n '1,240p' "$task_sdk/main.cpp"
printf '\n--- Current velocity/mode interface ---\n'
sed -n '1,280p' "$task_sdk/interface/user_command/dds_command_interface.hpp"
printf '\n--- Model / loop references ---\n'
grep -nE 'policy_path|onnx|S10PolicyRunner|SetDecimation|command =|forward_vel|side_vel|turnning|sc_q_|sc_wv_|SetJointCommand|create_wall_timer|LoopFunc|0.005|S10_RL_' "$task_sdk/run_policy/s10_policy_runner.hpp" "$task_sdk/state_machine/quadruped_wheel/rl_control_state.hpp" "$task_sdk/state_machine/quadruped_wheel/qw_state_machine.hpp" || true
printf '\n--- Model checksums ---\n'
find "$task_sdk/policy" /home/xxx/goai_embodied_future_material/install/s10_sdk_deploy -name '*.onnx' -type f -exec sha256sum {} +
printf '\n--- Service state (no mutation) ---\n'
systemctl show rl_deploy.service -p ActiveState -p MainPID -p UnitFileState
