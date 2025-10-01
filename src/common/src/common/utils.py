"""공통 유틸리티 함수"""
import rospy
import json
from typing import Tuple, Optional, Dict, Any

def safe_json_parse(json_string: str) -> Tuple[Optional[Dict], Optional[str]]:
    """안전한 JSON 파싱
    
    Args:
        json_string (str): JSON 문자열
        
    Returns:
        tuple: (파싱된 데이터, 에러 메시지)
    """
    try:
        data = json.loads(json_string)
        return data, None
    except json.JSONDecodeError as e:
        return None, str(e)
    except Exception as e:
        return None, f"Unexpected error: {str(e)}"

def create_response(success: bool, message: str, **kwargs) -> Dict[str, Any]:
    """표준 JSON 응답 생성
    
    Args:
        success (bool): 성공 여부
        message (str): 메시지
        **kwargs: 추가 필드
        
    Returns:
        dict: 응답 딕셔너리
    """
    response = {
        'success': success,
        'message': message,
        'timestamp': rospy.Time.now().to_sec()
    }
    response.update(kwargs)
    return response

def log_info(node_name: str, message: str):
    """표준 형식으로 info 로그 출력
    
    Args:
        node_name (str): 노드 이름
        message (str): 로그 메시지
    """
    rospy.loginfo(f"[{node_name}] {message}")

def log_error(node_name: str, message: str):
    """표준 형식으로 error 로그 출력
    
    Args:
        node_name (str): 노드 이름
        message (str): 로그 메시지
    """
    rospy.logerr(f"[{node_name}] {message}")

def log_warn(node_name: str, message: str):
    """표준 형식으로 warning 로그 출력
    
    Args:
        node_name (str): 노드 이름
        message (str): 로그 메시지
    """
    rospy.logwarn(f"[{node_name}] {message}")

def validate_json_fields(data: Dict, required_fields: list) -> Tuple[bool, Optional[str]]:
    """JSON 데이터의 필수 필드 검증
    
    Args:
        data (dict): 검증할 데이터
        required_fields (list): 필수 필드 리스트
        
    Returns:
        tuple: (유효 여부, 에러 메시지)
    """
    for field in required_fields:
        if field not in data:
            return False, f"Missing required field: {field}"
    return True, None