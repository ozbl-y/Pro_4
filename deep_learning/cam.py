import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import numpy as np

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

# 14x21 매트릭스 정의
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

# 행렬 크기
rows = len(matrix)
cols = len(matrix[0])

count = 0  # 프레임 수 카운트

while True:
    # 카메라에서 이미지 캡처
    im = picam2.capture_array()

    count += 1
    if count % 5 != 0:  # 프레임 건너뛰기 (속도 최적화)
        continue

    # 이미지 좌우 반전 (필요시)
    im = cv2.flip(im, -1)

    # YOLO 모델로 화재 감지
    results = model.predict(im, imgsz=320)  # 입력 이미지 크기 최적화
    boxes = results[0].boxes.data.cpu().numpy()  # 결과를 numpy 배열로 변환

    # 결과 데이터프레임으로 변환
    if len(boxes) > 0:  # 감지된 객체가 있을 경우에만 처리
        px = pd.DataFrame(boxes, columns=['x1', 'y1', 'x2', 'y2', 'confidence', 'class'])

        for index, row in px.iterrows():
            x1, y1, x2, y2 = int(row[0]), int(row[1]), int(row[2]), int(row[3])
            d = int(row[5])  # 클래스 인덱스
            c = class_list[d] if d < len(class_list) else 'Unknown'  # 클래스 이름

            # 화재 객체인 경우에만 경계 상자 및 클래스 텍스트 그리기
            if c == 'fire':  # 'fire' 객체가 감지된 경우
                # 경계 상자 그리기 (화재는 빨간색으로 표시)
                cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)
               
                # 클래스 텍스트 그리기
                text = f'{c}'
                cv2.putText(im, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                # 좌표 텍스트 그리기
                coordinates_text = f'({x1}, {y1}) ({x2}, {y2})'
                cv2.putText(im, coordinates_text, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # 매트릭스를 이용한 그리드 표현
    cell_height = im.shape[0] // rows
    cell_width = im.shape[1] // cols

    for i in range(rows):
        for j in range(cols):
            if not matrix[i][j]:  # False인 경우 사각형 그리기
                top_left = (j * cell_width, i * cell_height)
                bottom_right = ((j + 1) * cell_width, (i + 1) * cell_height)
                cv2.rectangle(im, top_left, bottom_right, (255, 0, 0), -1)  # 채워진 녹색 사각형

    # 결과 화면 표시
    cv2.imshow("Camera", im)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) == ord('q'):
        break

# 자원 해제
cv2.destroyAllWindows()
picam2.stop()
