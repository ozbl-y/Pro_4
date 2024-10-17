import time
import cv2
import numpy as np
import pandas as pd
from ultralytics import YOLO
from picamera2 import Picamera2
import matplotlib.pyplot as plt

# YOLO 모델 로드 (모델 파일 경로 지정)
model = YOLO('best1.pt')

# 클래스 리스트 읽기
with open("fire.txt", "r") as my_file:
    class_list = my_file.read().splitlines()

# Picamera2 초기화 및 설정
picam2 = Picamera2()
picam2.preview_configuration.main.size = (2200,1800)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

print("카메라가 켜졌습니다. 실시간 화재 감지 중...")

try:
    while True:
        # 카메라에서 이미지 캡처
        im = picam2.capture_array()

        # YOLO 모델로 화재 감지
        results = model.predict(im, imgsz=320)
        boxes = results[0].boxes.data.cpu().numpy()  # 결과를 numpy 배열로 변환

        # 화재가 감지된 경우 빨간 박스 표시
        if len(boxes) > 0:
            px = pd.DataFrame(boxes, columns=['x1', 'y1', 'x2', 'y2', 'confidence', 'class'])

            for index, row in px.iterrows():
                x1, y1, x2, y2 = int(row[0]), int(row[1]), int(row[2]), int(row[3])
                d = int(row[5])  # 클래스 인덱스
                c = class_list[d] if d < len(class_list) else 'Unknown'

                # 'fire' 객체가 감지된 경우에만 경계 상자 그리기
                if c == 'fire':
                    cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색 경계 상자
                    cv2.putText(im, f'Fire: {row[4]:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # BGR에서 RGB로 변환
        im_rgb = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

        # 결과 화면 표시 (Matplotlib 사용)
        plt.clf()  # 이전 이미지 삭제
        plt.imshow(im_rgb)
        plt.axis('off')  # 축 숨기기
        plt.show(block=False)
        plt.pause(0.001)  # 잠시 대기

        time.sleep(0.1)  # 프레임 속도 조절

except KeyboardInterrupt:
    print("프로그램이 중단되었습니다.")
except Exception as e:
    print(f"오류 발생: {e}")

# 자원 해제
plt.close()  # Matplotlib 창 닫기
picam2.stop()
