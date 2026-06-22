from enum import Enum

class ErrorCodes(Enum): 
    NO_ERROR = 0 
    NO_SCREWS = 1
    VISION_EXCEPTION = 2
    DL_FAIL = 3
    CAMMERA_CONNECTION_FAIL = 4
    LESS_THAN_FOUR_SCREWS_OR_DIST_WRONG = 5
    UNKOWN_EXCEPTION = 6