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
    AUGER_SPIN_UP   = 'AUGER_SPIN_UP' # bldc continuous spin [transition: immediate]
    DRILLING_DOWN   = 'DRILLING_DOWN' # linak runs out [transition: linak stop command]
    DRILLING_DWELL  = 'DRILLING_DWELL' # bldc continuous spin [transition: 15 seconds timer]
    AUGER_RETRACT   = 'AUGER_RETRACT'  # bldc spins other way, linak runs in [transition: linak stop command]
    SHIFT_TO_CHUTE  = 'SHIFT_TO_CHUTE' # bldc stops, stpper starts [transition: stepper timer stops; /topic ]
    WAIT_SEEDLING   = 'WAIT_SEEDLING' # 
    CHUTE_DOWN      = 'CHUTE_DOWN' # 
    CHUTE_RETRACT   = 'CHUTE_RETRACT'
    SHIFT_TO_AUGER  = 'SHIFT_TO_AUGER'
    COMPLETE        = 'COMPLETE'
    FAULT           = 'FAULT'
 