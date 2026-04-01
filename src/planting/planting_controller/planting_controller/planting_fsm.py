import enum
import time
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
 
 
# ============================================================
#  State definitions
# ============================================================
 
class State(enum.Enum):
    IDLE            = 'IDLE'
    AUGER_SPIN_UP   = 'AUGER_SPIN_UP'
    DRILLING_DOWN   = 'DRILLING_DOWN'
    DRILLING_DWELL  = 'DRILLING_DWELL'
    AUGER_RETRACT   = 'AUGER_RETRACT'
    SHIFT_TO_CHUTE  = 'SHIFT_TO_CHUTE'
    CHUTE_DOWN      = 'CHUTE_DOWN'
    WAIT_SEEDLING   = 'WAIT_SEEDLING'
    CHUTE_RETRACT   = 'CHUTE_RETRACT'
    SHIFT_TO_AUGER  = 'SHIFT_TO_AUGER'
    COMPLETE        = 'COMPLETE'
    FAULT           = 'FAULT'
 