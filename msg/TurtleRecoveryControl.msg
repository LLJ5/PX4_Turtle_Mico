# Internal leased request, consumed only by the DShot recovery backend.
uint64 timestamp
uint64 session_id
uint8 ACTION_STOP = 0
uint8 ACTION_PREPARE = 1
uint8 ACTION_DRIVE = 2
uint8 action
float32 throttle # [0, 0.8] DShot throttle fraction, not thrust, physical power or RPM
