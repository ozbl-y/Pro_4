import heapq
import serial
import time
import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import numpy as np

# 두 개의 시리얼 포트 설정
ser1 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # 첫 번째 포트
ser2 = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # 두 번째 포트
time.sleep(2)  # 포트가 열릴 때까지 잠시 대기

# Picamera2 초기화 및 설정
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# YOLO 모델 로드 (모델 파일 경로 지정)
model = YOLO('best1.pt')

# 클래스 리스트 읽기
with open("fire.txt", "r") as my_file:
    class_list = my_file.read().splitlines()

# A* 알고리즘을 사용하여 경로 탐색
def a_star_search(matrix, start, goal, fire=None):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    direction_labels = ['R', 'D', 'L', 'U']
    fire_directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    rows, cols = len(matrix), len(matrix[0])

    if fire:
        for dx, dy in fire_directions:
            fire_x, fire_y = fire[0] + dx, fire[1] + dy
            if 0 <= fire_x < rows and 0 <= fire_y < cols:
                matrix[fire_x][fire_y] = 'X'

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
   
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), start))
    came_from = {}
    g_score = {start: 0}
   
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            return reconstruct_optimal_path(matrix, came_from, start, goal, direction_labels, fire)
       
        for i, (dx, dy) in enumerate(directions):
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and matrix[neighbor[0]][neighbor[1]]:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = (current, direction_labels[i])
   
    return None

# 최적 경로를 재구성하는 함수
def reconstruct_optimal_path(matrix, came_from, start, goal, direction_labels, fire):
    matrix_visual = [['ㅁ' if cell else '  ' for cell in row] for row in matrix]
    current = goal
    while current != start:
        prev, direction = came_from[current]
        if matrix_visual[current[0]][current[1]] not in ('도', '불'):
            matrix_visual[current[0]][current[1]] = direction
        current = prev
    if matrix_visual[goal[0]][goal[1]] not in ('도', '불'):
        matrix_visual[goal[0]][goal[1]] = '도'
    if fire:
        matrix_visual[fire[0]][fire[1]] = '불'
    return matrix_visual

# 매트릭스 출력 함수
def print_matrix(matrix):
    for row in matrix:
        print(' '.join(str(cell).ljust(2) for cell in row))

# 출발 지점에서 가장 가까운 목표 지점을 찾는 함수
def closest_goal(start, goals):
    min_distance = float('inf')
    closest_goal = None
    for goal in goals:
        distance = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        if distance < min_distance:
            min_distance = distance
            closest_goal = goal
    return closest_goal

# 경로 누적을 위해 모든 출발 지점에서 목표 지점까지의 경로를 계산
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

# 최종 방향 매트릭스를 생성하는 함수
def generate_final_matrix(direction_count, matrix):
    final_matrix = [['O' for _ in range(len(matrix[0]))] for _ in range(len(matrix))]
   
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            for direction, count in direction_count[i][j].items():
                if count >= 1:
                    final_matrix[i][j] = direction
                    break
   
    return final_matrix

# 경로를 계산하고 최종 방향 매트릭스를 출력
def search_all_paths(matrix, starts, goals, direction_labels, fire=None):
    direction_count = accumulate_paths(matrix, starts, goals, direction_labels, fire)
    final_matrix = generate_final_matrix(direction_count, matrix)
   
    for goal in goals:
        final_matrix[goal[0]][goal[1]] = '도'
    if fire:
        final_matrix[fire[0]][fire[1]] = '불'
       
        rows, cols = len(matrix), len(matrix[0])
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                fire_x, fire_y = fire[0] + dx, fire[1] + dy
                if 0 <= fire_x < rows and 0 <= fire_y < cols:
                    final_matrix[fire_x][fire_y] = 'X'
   
    print_matrix(final_matrix)
   
    for i in range(len(final_matrix)):
        for j in range(len(final_matrix[0])):
            if final_matrix[i][j] in direction_labels:
                print(f"({i}, {j}) = {final_matrix[i][j]}")
            elif final_matrix[i][j] == ' ':
                print(f"({i}, {j}) = !")
            else:
                print(f"({i}, {j}) = {final_matrix[i][j]}")
       
    return final_matrix

# 결과를 추출하는 함수
def result(final_matrix):
    if final_matrix:
        results = [
            final_matrix[0][3],
            final_matrix[3][6],
            final_matrix[7][8],
            final_matrix[11][9],
            final_matrix[14][11],
            final_matrix[14][4],
            final_matrix[11][5],
            final_matrix[8][0],
            final_matrix[0][16],
            final_matrix[6][9],
            final_matrix[8][16],
            final_matrix[12][20],
        ]
        return results
    else:
        print("연산에 실패했습니다.")
        return []

# 아두이노에 데이터 전송
def send_data_to_arduinos(data):
    try:
        # 8개씩 묶어서 전송
        for i in range(0, len(data), 8):
            chunk = data[i:i+8]
            if len(chunk) < 8:  # 데이터가 8개 미만인 경우 패딩 추가
                chunk += [' '] * (8 - len(chunk))
            chunk_str = ''.join(chunk)  # 리스트를 문자열로 변환

            if i < 8:  # 첫 6개 묶음은 /dev/ttyACM0 포트로 전송
                ser1.write(chunk_str.encode())
                print(f"포트 /dev/ttyACM0로 전송된 데이터: {chunk_str}")
            else:  # 나머지 묶음은 /dev/ttyACM1 포트로 전송
                ser2.write(chunk_str.encode())
                print(f"포트 /dev/ttyACM1로 전송된 데이터: {chunk_str}")

        print("모든 데이터 전송 완료")
    except Exception as e:
        print(f"데이터 전송 오류: {e}")

# 매트릭스, 출발 지점 및 목표 지점 설정

matrix = [
    [True, True, True, True, True, True, True, True, True, True, True,True, True, True, True, True, True, True, True, True, True, True], #0
    [True, True, True, True, True, True, True, True, True, True, True,True, True, True, True, True, True, True, True, True, True, True],#1
    [True, True, True, True, True, True, False, False, False, False, False,True, True, True, True, True, True, True, True, True, True, True],#2
    [True, False, False, True, True, True, True, True, False, True, True,True, True, True, False,False,False,False, True, True, True, True],#3
    [True, True, True, True, True, True, True, True, False, True, True,True, True, True, True, True, True, True, True, True, True, True],#4
    [True, True, True, True, True, True, True, True, False, True, True,True, True, True, True, True, True, True, True, True, True, True],#5
    [True, True, True, True, True, True, False, False, False, True, True,True, True, True, True, True, True, True, True, True, True, True],#6
    [True, True, True, True, True, True, True, True, True, True,True,True,False, False, True, True, True, True, True, True, True, True],#7
    [True, True, True, True, True, True, True, True, True, True,True,True,False, False, True, True, True, True, False, False,False,False],#8
    [True, True, True,  False, False, False, False,False, False, True,True,True,False, False, True, True, True, True, True, True, True, True],#9
    [True,True, True,True, True, True, True, True,False, True, True, True, True, True, True, True, True, True, True, True, True, True],#10
    [True, True, True, True, True, True, True, True, False, True, True,True, True, True, True, True, True, True, True, True, True, True],#11
    [True, True, True, True, True, True, True, True, False, True, True,True, True, True, True, True, True, True, True, True, True, True],#12
    [True, True, False,False,False, False, False, False,False,False, True, True,True, False,False,False,False,False, True, True, True, True],#13
    [True, True, True, True, True,True, True, True, True, True, True, True,True, True, True, True, True, True, True, True,True, True],#14
]


# 출발 지점 정의
starts = [(i, j) for i in range(len(matrix)) for j in range(len(matrix[0])) if matrix[i][j]]

# 목표 지점 정의
goals = [(14, 0), (0, 21), (0, 4)]

# 화재 인식 상태를 저장하는 변수
fire_detected = False

# 화재 좌표를 카메라와 YOLO로 인식
while True:
    # 화재가 한 번 인식된 경우 더 이상 인식하지 않음
    if not fire_detected:
        # 카메라에서 이미지 캡처
        im = picam2.capture_array()
        
        # 이미지 좌우 반전 (필요시)
        im = cv2.flip(im, -1)

        # YOLO 모델로 화재 감지
        results = model.predict(im, imgsz=320)
        boxes = results[0].boxes.data.cpu().numpy()  # 결과를 numpy 배열로 변환

        # 화재가 감지된 셀 좌표 저장
        cell_coords = []

        if len(boxes) > 0:
            px = pd.DataFrame(boxes, columns=['x1', 'y1', 'x2', 'y2', 'confidence', 'class'])

            for index, row in px.iterrows():
                x1, y1, x2, y2 = int(row[0]), int(row[1]), int(row[2]), int(row[3])
                d = int(row[5])  # 클래스 인덱스
                c = class_list[d] if d < len(class_list) else 'Unknown'

                if c == 'fire':  # 'fire' 객체가 감지된 경우
                    # 화재 객체인 경우에만 경계 상자 및 클래스 텍스트 그리기
                    cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 경계 상자 (빨간색)

                    # 클래스 텍스트 그리기
                    text = f'{c}'
                    cv2.putText(im, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                    # 좌표 텍스트 그리기
                    coordinates_text = f'({x1}, {y1}) ({x2}, {y2})'
                    cv2.putText(im, coordinates_text, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    # 화재가 감지된 셀 계산
                    cell_height = im.shape[0] // len(matrix)
                    cell_width = im.shape[1] // len(matrix[0])

                    cell_x = x1 // cell_width
                    cell_y = y1 // cell_height
                    cell_coords.append((cell_y, cell_x))

            if cell_coords:
                avg_y = int(np.mean([coord[0] for coord in cell_coords]))
                avg_x = int(np.mean([coord[1] for coord in cell_coords]))

                # 평균 좌표로 화재 발생 지점 계산
                fire = (avg_y, avg_x)

                fire_detected = True  # 화재가 인식되었음을 기록

                try:
                    # 모든 출발 지점에서 가장 가까운 목표 지점까지의 경로 계산 및 방향 매트릭스 출력
                    final_matrix = search_all_paths(matrix, starts, goals, direction_labels=['R', 'D', 'L', 'U'], fire=fire)

                    # 결과 추출 및 아두이노로 데이터 전송
                    final_result = result(final_matrix)
                    send_data_to_arduinos(final_result)

                except Exception as e:
                    print(f"오류 발생: {e}")

                finally:
                    if ser1.is_open:
                        ser1.close()
                    if ser2.is_open:
                        ser2.close()

    # 결과 화면 표시
    cv2.imshow("Camera", im)

    if cv2.waitKey(1) == ord('q'):
        break

# 자원 해제
cv2.destroyAllWindows()
picam2.stop()