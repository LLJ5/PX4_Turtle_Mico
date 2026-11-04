# DShot backend state. Direction restoration is TX completion, not ESC acknowledgement.
uint64 timestamp
uint64 session_id
uint8 STATE_IDLE = 0
uint8 STATE_STOP_REVERSE = 1
uint8 STATE_REVERSING = 2
uint8 STATE_READY = 3
uint8 STATE_DRIVING = 4
uint8 STATE_STOP_RESTORE = 5
uint8 STATE_RESTORING = 6
uint8 state
uint8 result # 0: normal stop/cleanup; 1: watchdog or permission failure
bool busy
bool direction_dirty
float32 throttle
