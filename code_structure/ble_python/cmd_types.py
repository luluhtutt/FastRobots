from enum import Enum

class CMD(Enum):
    START_PID = 0
    STOP_PID = 1
    GET_PID_DATA = 2
    GET_PITCH_DATA = 3
    GET_ROLL_DATA = 4
    GET_YAW_DATA = 5
    GET_IMU_DATA = 6
    GET_TOF_DATA = 7
    START_PID_ORI = 8
    STOP_PID_ORI = 9
    GET_PID_DATA_ORI = 10
    START_MOVE = 11
    STOP_MOVE = 12
    GET_KF_DATA = 13