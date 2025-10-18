"""공통 상수 정의"""

# Topic 이름
class TOPICS:
    """ROS 토픽 이름"""
    MODE_BROAD = '/mode/broad'
    MODE_COMMAND = '/mode/command'
    MODE_STATUS = '/mode/status'

    WEB_CMD_VEL = '/web/cmd_vel'


    CMD_VEL = '/cmd_vel'
    EMERGENCY_STOP = '/emergency_stop'

class SERVICES:
    MODE_SRV = '/mode/req'

# Frame IDs
class FRAME_IDS:
    """TF Frame IDs"""
    MAP = 'map'
    ODOM = 'odom'
    BASE_LINK = 'base_link'
    LASER = 'laser'

# 기본 주기
class DEFAULT_RATES:
    """기본 퍼블리시 주기 (Hz)"""
    CONTROL = 10
    STATUS = 1
    SENSOR = 20