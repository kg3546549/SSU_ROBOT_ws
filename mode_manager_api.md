# Mode Control JSON Template

## 개요
ROS Mode Manager 서비스와 통신하기 위한 JSON 메시지 템플릿입니다.

---

## Service 정보

### Service
- **Name**: `/mode/req`
- **Type**: `ModeRequest`

---

## 1. GET - 현재 모드 조회

### 요청
```json
{
  "command": "get"
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

### 요청
```json
{
  "command": "set",
  "mode": 1,
  "speed": 0.5
}
```

### 파라미터
- `command`: "set" (필수)
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

### 요청
```json
{
  "command": "list"
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

### 잘못된 명령어
```json
{
  "success": false,
  "current_mode": 0,
  "mode_name": "IDLE",
  "message": "Unknown command: test"
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
rosservice call /mode/req "{request_data: '{\"command\": \"get\"}'}"
```

### SET 모드 변경
```bash
rosservice call /mode/req "{request_data: '{\"command\": \"set\", \"mode\": 1, \"speed\": 0.5}'}"
```

### LIST 모드 목록
```bash
rosservice call /mode/req "{request_data: '{\"command\": \"list\"}'}"
```

---

## Python 예시

### GET 요청
```python
import rospy
import json
from mode_manager.srv import ModeRequest

rospy.wait_for_service('/mode/req')
mode_req = rospy.ServiceProxy('/mode/req', ModeRequest)

request_data = json.dumps({"command": "get"})
response = mode_req(request_data)
result = json.loads(response.response_data)
print(result)
```

### SET 요청
```python
import rospy
import json
from mode_manager.srv import ModeRequest

rospy.wait_for_service('/mode/req')
mode_req = rospy.ServiceProxy('/mode/req', ModeRequest)

request_data = json.dumps({
    "command": "set",
    "mode": 1,
    "speed": 0.5
})
response = mode_req(request_data)
result = json.loads(response.response_data)
print(result)
```

### LIST 요청
```python
import rospy
import json
from mode_manager.srv import ModeRequest

rospy.wait_for_service('/mode/req')
mode_req = rospy.ServiceProxy('/mode/req', ModeRequest)

request_data = json.dumps({"command": "list"})
response = mode_req(request_data)
result = json.loads(response.response_data)
print(result)
```

---

## 명령어 목록

| 명령어 | 기능 | 필수 파라미터 |
|--------|------|--------------|
| `get` | 현재 모드 조회 | 없음 |
| `set` | 모드 변경 | `mode` |
| `list` | 모드 목록 조회 | 없음 |

---

## 주의사항

1. 모든 요청은 `/mode/req` 서비스를 통해 전송됩니다
2. `request_data` 필드에 JSON 문자열 형태로 전달
3. `command` 필드는 소문자로 전송 (대소문자 구분 안함)
4. 응답은 `response_data` 필드에 JSON 문자열로 반환
5. 유효하지 않은 모드 값 설정 시 에러 응답 반환