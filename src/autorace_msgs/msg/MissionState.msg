# Current state of the AutoRace mission state machine.

uint8 STANDBY=0
uint8 LANE_DRIVE=1
uint8 INTERSECTION=2
uint8 CONSTRUCTION=3
uint8 PARKING=4
uint8 LEVEL_CROSSING=5
uint8 TUNNEL=6
uint8 FINISH=7

std_msgs/Header header

uint8 state
# Free-form sub state of the active mission, e.g. "approach", "entering".
string detail
# Seconds spent in the current state.
float32 state_elapsed
# Seconds since the run started (first green light).
float32 run_elapsed
# Bit i is set when mission with state id i has been completed.
uint16 completed_mask
