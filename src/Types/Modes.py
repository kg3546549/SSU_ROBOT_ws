#!/usr/bin/env python3

from enum import Enum, IntEnum
#from typing import Dict, List, Optional

class RobotMode(Enum):
    """로봇 제어 모드 정의"""
    PATROL = "Patrol"           # 순찰 모드 (벽 따라가기, 장애물 회피)
    AUTO = "Auto"               # 자동 모드 (SLAM + Navigation)
    WEB_RC = "WEB_RC"           # 웹 원격 제어 모드
    MANUAL = "Manual"           # 수동 조작 모드 (조이스틱)
    STANDALONE = "Standalone"   # 독립 동작 모드 (통신 끊김 시 기본 동작)
    STANDBY = "Standby"   # 대기모드
    EMERGENCY = "Emergency"     # 긴급 정지 모드


class ModePriority(IntEnum):
    """모드 우선순위 (숫자가 클수록 높은 우선순위)"""
    PATROL = 1
    AUTO = 2  
    WEB_RC = 3
    MANUAL = 4
    STANDALONE = 5
    EMERGENCY = 6  # 최고 우선순위