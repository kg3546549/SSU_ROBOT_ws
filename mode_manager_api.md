# Mode Control JSON Template

## 개요
ROS Mode Manager 서비스와 통신하기 위한 JSON 메시지 템플릿입니다.

---

## 1. GET - 현재 모드 조회

### Service
- **Name**: `/mode/get`
- **Type**: `ModeRequest`

### 요청
```json
{
  "request_data": ""
}
```

### 응답 예시
```json
{
  "success": true,
  "current_mode": 0,
  "mode_name": "IDLE",
  "message": "Current mode retrieved"
}
```

---

## 2. SET - 모드 변경

### Service
- **Name**: `/mode/set`
- **Type**: `ModeRequest`

### 요청
```json
{
  "request_data": "{\"mode\": 1, \"speed\": 0.5}"
}
```

### 파라미터
- `mode`: 변경할 모드 값 (필수)
- `speed`: 속도 설정 (선택, 기본값: 0.5)

### 응답 예시
```json
{
  "success": true,
  "current_mode": 1,
  "mode_name": "AUTO_PATROL",
  "message": "Mode changed successfully"
}
```

---

## 3. LIST - 사용 가능한 모드 목록 조회

### Service
- **Name**: `/mode/list`
- **Type**: `ModeRequest`

### 요청
```json
{
  "request_data": ""
}
```

### 응답 예시
```json
{
  "success": true,
  "modes": [
    {"value": 0, "name": "IDLE"},
    {"value": 1, "name": "AUTO_PATROL"},
    {"value": 2, "name": "AUTO_NAVIGATION"},
    {"value": 3, "name": "KEYBOARD"},
    {"value": 4, "name": "WEB_JOY"},
    {"value": 5, "name": "JOY"},
    {"value": 99, "name": "EMERGENCY"}
  ]
}
```

---

## 4. 에러 응답

### 잘못된 요청
```json
{
  "success": false,
  "current_mode": 0,
  "mode_name": "IDLE",
  "message": "Invalid request"
}
```

### JSON 파싱 에러
```json
{
  "success": false,
  "current_mode": 0,
  "mode_name": "IDLE",
  "message": "Invalid JSON format"
}
```

---

## 로봇 모드 정의

| Mode 값 | 이름 | 설명 |
|---------|------|------|
| 0 | IDLE | 대기 상태 |
| 1 | AUTO_PATROL | 자동 순찰 모드 |
| 2 | AUTO_NAVIGATION | 자동 내비게이션 모드 |
| 3 | KEYBOARD | 키보드 제어 모드 |
| 4 | WEB_JOY | 웹 조이스틱 제어 모드 |
| 5 | JOY | 조이스틱 제어 모드 |
| 99 | EMERGENCY | 비상 정지 모드 |

---

## ROS Service 호출 예시

### GET 모드 조회
```bash
rosservice call /mode/get "{request_data: ''}"
```

### SET 모드 변경
```bash
rosservice call /mode/set "{request_data: '{\"mode\": 1, \"speed\": 0.5}'}"
```

### LIST 모드 목록
```bash
rosservice call /mode/list "{request_data: ''}"
```

---

## Python 예시

### GET 요청
```python
import rospy
from mode_manager.srv import ModeRequest

rospy.wait_for_service('/mode/get')
get_mode = rospy.ServiceProxy('/mode/get', ModeRequest)
response = get_mode("")
```

### SET 요청
```python
import rospy
import json
from mode_manager.srv import ModeRequest

rospy.wait_for_service('/mode/set')
set_mode = rospy.ServiceProxy('/mode/set', ModeRequest)

request_data = json.dumps({"mode": 1, "speed": 0.5})
response = set_mode(request_data)
```

### LIST 요청
```python
import rospy
from mode_manager.srv import ModeRequest

rospy.wait_for_service('/mode/list')
list_modes = rospy.ServiceProxy('/mode/list', ModeRequest)
response = list_modes("")
```

---

## 서비스 목록

| 서비스 이름 | 기능 | 입력 필요 |
|------------|------|----------|
| `/mode/get` | 현재 모드 조회 | ❌ (빈 문자열) |
| `/mode/set` | 모드 변경 | ✅ (mode, speed) |
| `/mode/list` | 모드 목록 조회 | ❌ (빈 문자열) |

---

## 주의사항

1. 모든 서비스는 `ModeRequest` 타입을 사용합니다
2. 요청 데이터는 `request_data` 필드에 JSON 문자열로 전달
3. 응답 데이터는 `response_data` 필드에 JSON 문자열로 반환
4. 유효하지 않은 모드 값 설정 시 에러 응답 반환