"""로봇 모드 정의 및 유틸리티"""
from enum import Enum

class RobotMode(Enum):
    """로봇 동작 모드"""
    IDLE = 0
    AUTO_PATROL = 1
    AUTO_NAVIGATION = 2
    KEYBOARD = 3
    WEB_JOY = 4
    JOY = 5
    EMERGENCY = 99

def get_mode_by_value(value):
    """값으로 모드 찾기
    
    Args:
        value (int): 모드 값
        
    Returns:
        RobotMode or None: 찾은 모드 또는 None
    """
    for mode in RobotMode:
        if mode.value == value:
            return mode
    return None

def get_mode_by_name(name):
    """이름으로 모드 찾기
    
    Args:
        name (str): 모드 이름
        
    Returns:
        RobotMode or None: 찾은 모드 또는 None
    """
    try:
        return RobotMode[name.upper()]
    except KeyError:
        return None

def is_auto_mode(mode):
    """자율 주행 모드 여부
    
    Args:
        mode (RobotMode): 확인할 모드
        
    Returns:
        bool: 자율 주행 모드이면 True
    """
    return mode in [RobotMode.AUTO_PATROL, RobotMode.AUTO_NAVIGATION]

def is_manual_mode(mode):
    """수동 제어 모드 여부
    
    Args:
        mode (RobotMode): 확인할 모드
        
    Returns:
        bool: 수동 제어 모드이면 True
    """
    return mode in [RobotMode.KEYBOARD, RobotMode.WEB_JOY, RobotMode.JOY]

def get_all_modes():
    """모든 모드 목록 반환
    
    Returns:
        list: [{value: int, name: str}, ...]
    """
    return [
        {'value': mode.value, 'name': mode.name}
        for mode in RobotMode
    ]