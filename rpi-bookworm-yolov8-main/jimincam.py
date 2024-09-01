import heapq
import serial
import time
import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import cvzone
import numpy as np

# Initialize the camera and YOLO model
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Load YOLO model
model = YOLO('yolov8n.pt')  # Ensure this path is correct and model file is available

# Load class list
with open("/home/raspi/rpi-bookworm-yolov8-main/fire.txt", "r") as my_file:
    data = my_file.read()
    class_list = data.split("\n")

# Initialize serial communication
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # USB 포트 사용
time.sleep(2)  # 포트가 열릴 때까지 잠시 대기

def a_star_search(matrix, start, goal, fire=None):
    # 이동 방향 정의
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    direction_labels = ['R', 'D', 'L', 'U']
   
    rows, cols = len(matrix), len(matrix[0])

    # 불이 퍼진 영역을 장애물로 처리 (5x5 영역)
    if fire:
        fire_x, fire_y = fire
        for dx in range(-2, 3):  # -2 to 2 for 5x5 area
            for dy in range(-2, 3):
                new_x, new_y = fire_x + dx, fire_y + dy
                if 0 <= new_x < rows and 0 <= new_y < cols:
                    matrix[new_x][new_y] = False  # 불이 퍼진 위치는 장애물로 표시

    # 휴리스틱 함수 정의 (Manhattan 거리 기준)
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
   
    open_set = []  # 열린 목록
    heapq.heappush(open_set, (0 + heuristic(start, goal), start))
    came_from = {}  # 경로 추적을 위한 사전
    g_score = {start: 0}  # 시작 지점으로부터의 최소 경로 비용
   
    while open_set:
        current = heapq.heappop(open_set)[1]  # 열린 목록에서 가장 낮은 비용의 노드 추출
        if current == goal:
            return reconstruct_optimal_path(matrix, came_from, start, goal, direction_labels)
       
        for i, (dx, dy) in enumerate(directions):
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                # 이웃 노드가 유효하고 벽이 아닌 경우
                if matrix[neighbor[0]][neighbor[1]]:
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))
                        came_from[neighbor] = (current, direction_labels[i])
   
    return None

# 최적 경로 재구성 함수
def reconstruct_optimal_path(matrix, came_from, start, goal, direction_labels):
    matrix_visual = [['ㅁ' if cell else '  ' for cell in row] for row in matrix]
    current = goal
    while current != start:
        prev, direction = came_from[current]
        if matrix_visual[current[0]][current[1]] not in ('도', '불'):
            matrix_visual[current[0]][current[1]] = direction
        current = prev
    if matrix_visual[goal[0]][goal[1]] not in ('도', '불'):
        matrix_visual[goal[0]][goal[1]] = '도'
    return matrix_visual

# 매트릭스 출력 함수
def print_matrix(matrix):
    for row in matrix:
        print(' '.join(str(cell).ljust(2) for cell in row))

# 시작 지점과 목표 지점 간의 최단 거리 계산
def closest_goal(start, goals):
    min_distance = float('inf')
    closest_goal = None
    for goal in goals:
        distance = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        if distance < min_distance:
            min_distance = distance
            closest_goal = goal
    return closest_goal

# 모든 출발 지점에서 가장 가까운 목표 지점까지의 경로를 모두 계산하고 방향 개수를 누적
def accumulate_paths(matrix, starts, goals, direction_labels, fire=None):
    rows, cols = len(matrix), len(matrix[0])
    direction_count = [[{} for _ in range(cols)] for _ in range(rows)]
   
    for start in starts:
        closest = closest_goal(start, goals)
        path_matrix = a_star_search([row[:] for row in matrix], start, closest, fire)
       
        if path_matrix:
            for i, row in enumerate(path_matrix):
                for j, cell in enumerate(row):
                    if cell in direction_labels:
                        if cell not in direction_count[i][j]:
                            direction_count[i][j][cell] = 0
                        direction_count[i][j][cell] += 1
   
    return direction_count

# 최종 방향 매트릭스 생성 함수
def generate_final_matrix(direction_count, matrix):
    final_matrix = [['  ' for _ in range(len(matrix[0]))] for _ in range(len(matrix))]
   
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            for direction, count in direction_count[i][j].items():
                if count >= 1:
                    final_matrix[i][j] = direction
                    break
   
    return final_matrix

# 모든 출발 지점에서 가장 가까운 목표 지점까지의 경로를 계산하고 최종 방향 매트릭스를 출력
def search_all_paths(matrix, starts, goals, direction_labels, fire=None):
    direction_count = accumulate_paths(matrix, starts, goals, direction_labels, fire)
    final_matrix = generate_final_matrix(direction_count, matrix)
   
    for goal in goals:
        final_matrix[goal[0]][goal[1]] = '도'
    if fire:
        final_matrix[fire[0]][fire[1]] = '불'
        for dx in range(-2, 3):  # -2 to 2 for 5x5 area
            for dy in range(-2, 3):
                fire_x, fire_y = fire[0] + dx, fire[1] + dy
                if 0 <= fire_x < len(matrix) and 0 <= fire_y < len(matrix[0]):
                    final_matrix[fire_x][fire_y] = 'X'
   
    print_matrix(final_matrix)
   
    return final_matrix

# 결과 추출 함수
def result(final_matrix):
    if final_matrix:
        results = [
            final_matrix[4][1],
            final_matrix[3][4],
            final_matrix[4][6],
            final_matrix[3][8],
            final_matrix[8][8],
            final_matrix[7][6],
            final_matrix[8][4],
            final_matrix[7][2],
            final_matrix[12][1],
            final_matrix[11][4],
            final_matrix[12][6],
            final_matrix[11][8],
        ]
        return results
    else:
        print("연산에 실패했습니다.")

# 매트릭스와 출발 지점, 목표 지점 설정
matrix = [[True] * 22 for _ in range(14)]

# 출발 지점 정의
starts = [(i, j) for i in range(len(matrix)) for j in range(len(matrix[0])) if matrix[i][j]]

# 목표 지점 정의
goals = [(0, 6), (0, 21), (13, 0)]

# 카메라에서 불의 위치를 감지하는 루프 시작
count = 0
while True:
    im = picam2.capture_array()
   
    count += 1
    if count % 3 != 0:
        continue
   
    im = cv2.flip(im, -1)
    results = model.predict(im)
    a = results[0].boxes.data
    px = pd.DataFrame(a).astype("float")
   
    fire_detected = False
   
    for index, row in px.iterrows():
        x1 = int(row[0])
        y1 = int(row[1])
        x2 = int(row[2])
        y2 = int(row[3])
        d = int(row[5])
        c = class_list[d]
       
        if c == 'fire':  # Assuming 'fire' is the label for fire detection
            fire_detected = True
       
        cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cvzone.putTextRect(im, f'{c}', (x1, y1), 1, 1)
   
    # If fire is detected, use fixed coordinates (4, 4) and calculate the path
    if fire_detected:
        fire = (4, 4)  # Set fire coordinates to (4, 4)
        final_matrix = search_all_paths(matrix, starts, goals, direction_labels=['R', 'D', 'L', 'U'], fire=fire)

        # 결과 출력 및 아두이노로 전송
        final_result = result(final_matrix)
        if final_result:
            print("결과:", final_result)
            for res in final_result:
                if res in ['R', 'D', 'L', 'U']:
                    ser.write(res.encode())
                else:
                    ser.write(' '.encode())  # 방향이 없는 경우 공백으로 전송
        else:
            print("연산에 실패했습니다.")
   
    cv2.imshow("Camera", im)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
