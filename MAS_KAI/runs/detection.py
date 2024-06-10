from ultralytics import YOLO

# YOLOv8 모델을 로드합니다. 'best.pt'는 사전에 학습된 가중치 파일입니다.
model = YOLO('C:/Users/user/Desktop/MAS_KAI/runs/detect/train4/weights/best.pt')

def detect_objects(img):
    # 이미지에서 객체를 검출합니다. 'stream=True'는 결과를 스트림 형식으로 반환합니다.
    results = model(img, stream=True)
    
    # results를 리스트로 변환합니다. (스트림 형식이므로 리스트로 변환하여 다룹니다.)
    results = list(results)
    class_ids = []  # 검출된 객체의 클래스 ID를 저장할 리스트를 초기화합니다.

    for result in results:
        # 결과를 시각화하여 주석이 달린 프레임을 생성합니다.
        # 각 검출 결과를 반복하며, 주석이 달린 프레임(객체 검출 박스와 레이블이 포함된 이미지)을 생성합니다.
        annotated_frame = result.plot()  
        
        # 검출된 객체의 클래스 ID와 신뢰도(confidence) 점수를 순회합니다.
        for cls_id_tensor, conf in zip(result.boxes.cls, result.boxes.conf):
            conf_value = conf.item()  # 신뢰도 점수를 파이썬 float 형식으로 변환합니다.
            if conf_value >= 0.65:  # 신뢰도가 0.6 이상인 경우에만 처리합니다.
                cls_id = int(cls_id_tensor.item())  # 클래스 ID를 파이썬 정수 형식으로 변환합니다.
                class_ids.append(cls_id)  # 클래스 ID를 리스트에 추가합니다.
    
    # 주석이 달린 프레임과 검출된 클래스 ID 리스트를 반환합니다.
    return annotated_frame, class_ids
