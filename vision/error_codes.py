from enum import Enum

class ErrorCodes(Enum): 
    NO_ERROR = 0 
    NO_SCREWS = 1
    VISION_EXCEPTION = 2
    DL_FAIL = 3