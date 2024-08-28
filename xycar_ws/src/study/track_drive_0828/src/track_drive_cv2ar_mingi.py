import cv2
import numpy as np

def detect_aruco_tags(frame):
    # AR 태그의 사전 정의 (여기서는 4x4 태그 사용)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()

    # AR 태그 감지
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    return corners, ids

def draw_aruco_bounding_boxes(frame, corners, ids):
    # AR 태그가 감지되면 화면에 표시
    if corners:
        for i in range(len(corners)):
            corner = corners[i][0]
            id = ids[i][0]

            # AR 태그의 코너를 다각형으로 그리기
            cv2.polylines(frame, [np.int32(corner)], True, (0, 255, 0), 2)

            # 태그 아이디를 표시
            cX, cY = np.int32(corner).mean(axis=0)
            cv2.putText(frame, f'ID: {id}', (int(cX), int(cY)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

def main():
    # 비디오 캡처 객체 생성
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # AR 태그 감지
        corners, ids = detect_aruco_tags(frame)

        # AR 태그의 위치에 사각형 그리기
        draw_aruco_bounding_boxes(frame, corners, ids)

        # 결과를 화면에 표시
        cv2.imshow('AR Tag Detection', frame)

        # 'q'를 눌러서 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 캡처 객체와 창을 닫기
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

