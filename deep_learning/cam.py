import heapq
import serial
import time
import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import numpy as np

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
model = YOLO('best1.pt')
with open("fire.txt", "r") as my_file:
    class_list = my_file.read().splitlines()

# 그리드 기반 매트릭스 설정
# 매트릭스 예시
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

def a_star_search(graph, start, goal, heuristic_func, blocked):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic_func(start, goal), start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            return reconstruct_path(came_from, start, goal)

        for neighbor in graph.get(current, []):
            if neighbor in blocked:
                continue

            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic_func(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current

    return None

def reconstruct_path(came_from, start, goal):
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def find_closest_goal(graph, start, goals, blocked):
    closest_path = None
    min_distance = float('inf')
    for goal in goals:
        path = a_star_search(graph, start, goal, heuristic, blocked)
        if path:
            distance = len(path)
            if distance < min_distance:
                min_distance = distance
                closest_path = path
    return closest_path

def print_first_direction(start, path):
    if path:
        next = path[0]
        if next[0] < start[0]:
            return 'U'
        elif next[0] > start[0]:
            return 'D'
        elif next[1] < start[1]:
            return 'L'
        elif next[1] > start[1]:
            return 'R'
    return 'X'

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

    # 화재가 감지되었을 때 15초간 대기하고, 이후 다시 감지 시도
    if not fire_detected or current_time - last_detection_time >= 15:
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
                    cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    fire_text = f"({fire_x}, {fire_y})"
                    cv2.putText(im, fire_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    print(f"화재 감지 위치: {fire_text}")

            # 화재 감지를 한 번만 수행하기 위해 플래그 및 시간 설정
            fire_detected = True
            last_detection_time = current_time
            captured_image = im.copy()  # 화재 감지 시 이미지 복사본 저장

            # 화재 좌표 계산 및 추가 장애물로 간주
            if cell_coords:
                fire_y = int(np.mean([coord[0] for coord in cell_coords]))
                fire_x = int(np.mean([coord[1] for coord in cell_coords]))
                fire = (fire_y, fire_x)
                print(f"평균 화재 위치: ({fire_x}, {fire_y})")

                # 장애물 처리
                blocked = {(fire_y + dy, fire_x + dx) for dx in range(-1, 2) for dy in range(-1, 2)
                           if (0 <= fire_y + dy < 9 and 0 <= fire_x + dx < 11)}

                # 각 출발 지점에서 가장 가까운 목표 지점까지의 경로 계산
                graph = create_graph(matrix)
                directions = []
                for start in start_points:
                    closest_path = find_closest_goal(graph, start, goals, blocked)
                    if start in blocked:
                        directions.append('X')
                    else:
                        first_direction = print_first_direction(start, closest_path)
                        directions.append(first_direction)

                # 아두이노로 방향 전송
                send_data_to_arduinos(directions)

    # 감지 결과 표시 (화재 감지 시 저장된 이미지가 계속 표시됨)
    cv2.imshow("Camera", captured_image if captured_image is not None else im)
    if cv2.waitKey(1) == ord('q'):
        break

# 자원 해제
cv2.destroyAllWindows()
picam2.stop()
ser1.close()
ser2.close()
