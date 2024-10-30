import heapq
import serial
import time
import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import numpy as np
from datetime import datetime  # 현재 시간 출력을 위한 라이브러리 추가

# 시리얼 포트 설정
ser1 = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
ser2 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

# Picamera2 초기화 및 설정
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# YOLO 모델 로드 및 클래스 리스트 읽기
model = YOLO('best.pt')
with open("fire.txt", "r") as my_file:
    class_list = my_file.read().splitlines()

# 그리드 기반 매트릭스 설정
matrix = [
    [False, False, False, True, False, False, False, False, False, False, False], #0
    [False, True, True, True, True, True, True, True, True, True, True], #1
    [False, True, False, True, False, False, False, True, False, False, False], #2
    [False, True, True, True, True, True, True, True, False, False, False], #3
    [False, True, False, True, False, False, False, True, True, True, False], #4
    [False, True, True, True, True, True, True, True, False, True, False], #5
    [False, True, False, True, False, False, True, False, False, True, False], #6
    [False, True, True, True, True, True, True, True, True, True, False], #7
    [False, False, False, False, False, False, True, False, False, False, False], #8
]

# 출발 지점과 목표 설정
start_points = [(1, 3), (3, 3), (5, 3), (7, 3), (7, 1), (5, 1), (3, 1), (1, 1),(1, 7), (3, 7), (4, 7), (5, 6), (7, 6), (7, 9), (4, 9)]
goals = [(1, 10), (0, 3), (8,6)]

# 휴리스틱 함수 (맨해튼 거리)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* 알고리즘 및 그래프 생성 함수
def create_graph(matrix):
    rows, cols = len(matrix), len(matrix[0])
    graph = {}
    for i in range(rows):
        for j in range(cols):
            if matrix[i][j]:
                neighbors = []
                for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and matrix[ni][nj]:
                        neighbors.append((ni, nj))
                graph[(i, j)] = neighbors
    return graph

# A* 알고리즘 적용
def a_star_search(graph, start, goal, heuristic_func, blocked):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic_func(start, goal), start, []))  # 경로를 리스트로 추가
    all_paths = []  # 여러 경로를 저장
    g_score = {start: 0}
   
    while open_set:
        _, current, path = heapq.heappop(open_set)
        path = path + [current]

        if current == goal:
            all_paths.append(path)

        for neighbor in graph[current]:
            if neighbor in blocked:
                continue  # 장애물 주변인 경우 건너뛰기

            tentative_g_score = g_score[current] + 1  # 각 간선의 가중치는 1
            if neighbor not in g_score or tentative_g_score <= g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic_func(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor, path))

    return all_paths if all_paths else None

# 최적 경로를 다중으로 재구성
def find_closest_goal(graph, start, goals, blocked, blocked_around_fire, fire_coords):
    # 시작점이 불 주변 8칸에 속하는 경우 불 좌표만 장애물로 간주
    if start in blocked_around_fire:
        blocked = {fire_coords}  # 불 좌표 하나만 장애물로 설정
    elif start in blocked:
        return None  # 시작점이 불 좌표에 해당하면 경로를 찾지 않음
   
    closest_paths = []
    min_distance = float('inf')
   
    for goal in goals:
        paths = a_star_search(graph, start, goal, heuristic, blocked)
        if paths:
            for path in paths:
                distance = len(path)
                if distance < min_distance:
                    min_distance = distance
                    closest_paths = [path]  # 새로운 최단 경로
                elif distance == min_distance:
                    closest_paths.append(path)  # 동일한 거리의 경로 추가
   
    return closest_paths if closest_paths else None

# 경로를 계산하고 첫 번째 방향을 출력
def print_first_directions(start, paths):
    if not paths:
        return 'X'  # 경로가 없으면 'X' 출력

    directions = set()
    for path in paths:
        if len(path) > 1:
            next_step = path[1]
            if next_step[0] < start[0]:
                directions.add('U')  # 위
            elif next_step[0] > start[0]:
                directions.add('D')  # 아래
            elif next_step[1] < start[1]:
                directions.add('L')  # 왼쪽
            elif next_step[1] > start[1]:
                directions.add('R')  # 오른쪽
    return list(directions) if directions else [' ']

# 방향을 변환하는 함수 정의
def convert_directions(group):
    result = []
    for directions in group:
        if directions == ['D', 'R'] or directions == ['R', 'D']:
            result.append('A')
        elif directions == ['D', 'L'] or directions == ['L', 'D']:
            result.append('B')
        elif directions == ['R', 'L'] or directions == ['L', 'R']:
            result.append('C')
        elif directions == ['U', 'D'] or directions == ['D', 'U']:
            result.append('D')
        elif directions == ['U', 'L'] or directions == ['L', 'U']:
            result.append('E')
        elif directions == ['U', 'R'] or directions == ['R', 'U']:
            result.append('F')
        elif isinstance(directions, list):
            result.append(directions[0])
        else:
            result.append(directions)
    return result

# 데이터 전송 함수
def send_data_to_arduinos(data):
    try:
        for i in range(0, len(data), 8):
            chunk = data[i:i+8]
            if len(chunk) < 8:
                chunk += [' '] * (8 - len(chunk))
            chunk_str = ''.join(chunk)
            if i < 8:
                ser1.write(chunk_str.encode())
                print(f"포트 /dev/ttyACM0로 전송된 데이터: {chunk_str}")
            else:
                ser2.write(chunk_str.encode())
                print(f"포트 /dev/ttyACM1로 전송된 데이터: {chunk_str}")
        print("모든 데이터 전송 완료")
    except Exception as e:
        print(f"데이터 전송 오류: {e}")

# 화재 감지 플래그
fire_detected = False
last_detection_time = time.time()
captured_image = None

while True:
    current_time = time.time()

    # 화재가 감지되었을 때 5초간 대기하고, 이후 다시 감지 시도
    if not fire_detected or current_time - last_detection_time >= 5:
        # 카메라에서 이미지 캡처 후 YOLO로 화재 감지
        im = picam2.capture_array()
        im = cv2.flip(im, -1)

        results = model.predict(im, imgsz=320)
        boxes = results[0].boxes.data.cpu().numpy()

        # 화재 위치 좌표 추출 및 빨간 박스 표시
        cell_coords = []
        if len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2, _, cls_id = map(int, box[:6])
                class_name = class_list[cls_id]
                if class_name == 'fire':
                    cell_width = im.shape[1] // 11
                    cell_height = im.shape[0] // 9
                    fire_x = x1 // cell_width
                    fire_y = y1 // cell_height
                    cell_coords.append((fire_y, fire_x))
                    
                    # 빨간 박스 및 좌표 텍스트 출력
                    fire_width = x2 - x1  # 불의 너비
                    fire_height = y2 - y1  # 불의 높이
                    fire_size = fire_width * fire_height  # 불의 크기
                    current_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # 현재 시각

                    # 줄바꿈을 사용하여 여러 줄로 출력
                    fire_text = f"Time: {current_time_str}\nSize: {fire_size}\nCoord: ({fire_y}, {fire_x})"

                    # 화면 출력 (파란색으로 변경: (255, 0, 0))
                    cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색 박스
                    for i, line in enumerate(fire_text.split('\n')):
                        cv2.putText(im, line, (x1, y1 - 10 - i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)  # 파란색 텍스트

                    # 터미널 출력
                    print(f"화재 감지:\n{fire_text}")


                    

            # 화재 감지를 한 번만 수행하기 위해 플래그 및 시간 설정
            fire_detected = True
            last_detection_time = current_time
            captured_image = im.copy()  # 화재 감지 시 이미지 복사본 저장

            # 화재 좌표 계산 및 추가 장애물로 간주
            if len(cell_coords) >= 2:  # 두 개의 화재가 감지된 경우
                fire1_y, fire1_x = cell_coords[0]
                fire2_y, fire2_x = cell_coords[1]
                fire_coords = (fire1_y, fire1_x)

                # 두 개의 화재 좌표만 장애물로 설정
                blocked = {(fire1_y, fire1_x), (fire2_y, fire2_x)}
            else:
                # 하나의 화재만 감지된 경우
                fire_y = int(np.mean([coord[0] for coord in cell_coords]))
                fire_x = int(np.mean([coord[1] for coord in cell_coords]))
                fire_coords = (fire_y, fire_x)
                blocked = {(fire_y, fire_x)}

            # 불 주변 8칸 추가
            blocked_around_fire = {(fire_y + dy, fire_x + dx) for dx in range(-1, 2) for dy in range(-1, 2) if (0 <= fire_y + dy < 9 and 0 <= fire_x + dx < 11)}
            blocked |= blocked_around_fire  # 불 좌표와 그 주변 8칸을 모두 장애물로 간주

            # 각 출발 지점에서 가장 가까운 목표 지점까지의 경로 계산
            graph = create_graph(matrix)
            directions = []
            for start in start_points:
                closest_paths = find_closest_goal(graph, start, goals, blocked, blocked_around_fire, fire_coords)
                first_directions = print_first_directions(start, closest_paths)
                directions.append(first_directions)

            # 변환된 방향을 아두이노로 전송
            converted_directions = convert_directions(directions)
            send_data_to_arduinos(converted_directions)

    # 감지 결과 표시 (화재 감지 시 저장된 이미지가 계속 표시됨)
    cv2.imshow("Camera", captured_image if captured_image is not None else im)
    if cv2.waitKey(1) == ord('q'):
        break

# 자원 해제
cv2.destroyAllWindows()
picam2.stop()
ser1.close()
ser2.close()
