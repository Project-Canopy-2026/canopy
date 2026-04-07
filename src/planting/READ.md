Watch outgoing commands:
  ros2 topic echo /arduino_cmd
  ros2 topic echo /linak_cmd
  ros2 topic echo planting_state
  ros2 topic echp /linak_status

  Fake the hardware responses — each time the FSM sends a command and waits,
   publish the expected reply manually:

  # After LINAK_1 moves (DRILLING_DOWN and AUGER_RETRACT):
  ros2 topic pub --once /linak_status std_msgs/msg/String '{data:
  "DONE:LINAK1"}'

  # After LINAK_2 moves (CHUTE_DOWN and CHUTE_RETRACT):
  ros2 topic pub --once /linak_status std_msgs/msg/String '{data:
  "DONE:LINAK2"}'

  # After stepper moves (SHIFT_TO_CHUTE and SHIFT_TO_AUGER):
  ros2 topic pub --once /arduino_status std_msgs/msg/String '{data:
  "DONE:STEPPER"}'

  Expected flow:
  1. p in test node → FSM enters AUGER_SPIN_UP → DRILLING_DOWN, /arduino_cmd
   and /linak_cmd fire
  2. Publish DONE:LINAK1 → FSM enters DRILLING_DWELL (10s timer, waits
  automatically)
  3. After 10s → FSM enters AUGER_RETRACT, commands fire
  4. Publish DONE:LINAK1 → SHIFT_TO_CHUTE, commands fire
  5. Publish DONE:STEPPER → WAIT_SEEDLING
  6. s in test node → CHUTE_DOWN, command fires
  7. Publish DONE:LINAK2 → CHUTE_RETRACT, command fires
  8. Publish DONE:LINAK2 → SHIFT_TO_AUGER, command fires
  9. Publish DONE:STEPPER → COMPLETE → IDLE