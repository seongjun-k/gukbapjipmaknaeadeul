"""체스보드 카메라 캘리브레이션 CLI.

두 모드:
- 라이브: 장치를 열어 스페이스로 프레임 수집, c로 계산.
- 이미지: --images 디렉토리의 파일들로 바로 계산 (GUI 없는 환경용).

결과(K, dist)를 --out(.npz)에 저장.
"""
import argparse
import glob
import os

import cv2
import numpy as np


def _find_corners(gray, pattern_size):
    return cv2.findChessboardCorners(gray, pattern_size)


def calibrate_from_images(image_paths, pattern_size, square_size_m):
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_m

    objpoints = []
    imgpoints = []
    img_shape = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]
        found, corners = _find_corners(gray, pattern_size)
        if not found:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)

    if len(objpoints) < 3:
        raise RuntimeError(f"체스보드 검출 성공 이미지가 부족합니다 ({len(objpoints)}장)")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )
    return K, dist, ret


def calibrate_from_device(device, pattern_size, square_size_m):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"카메라 장치를 열 수 없습니다: {device}")

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_m

    objpoints = []
    imgpoints = []
    img_shape = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print("스페이스: 프레임 수집 / c: 계산 / q: 취소")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_shape = gray.shape[::-1]
            found, corners = _find_corners(gray, pattern_size)
            disp = frame.copy()
            if found:
                cv2.drawChessboardCorners(disp, pattern_size, corners, found)
            cv2.putText(
                disp, f"collected: {len(objpoints)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
            )
            cv2.imshow("calibrate", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" ") and found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners)
                print(f"프레임 수집: {len(objpoints)}장")
            elif key == ord("c"):
                break
            elif key == ord("q"):
                raise RuntimeError("사용자가 취소했습니다")
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if len(objpoints) < 3:
        raise RuntimeError(f"체스보드 검출 성공 프레임이 부족합니다 ({len(objpoints)}장)")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )
    return K, dist, ret


def main():
    parser = argparse.ArgumentParser(description="체스보드 카메라 캘리브레이션")
    parser.add_argument("--device", type=int, default=0, help="라이브 모드: 카메라 장치 번호")
    parser.add_argument("--images", type=str, default=None, help="이미지 디렉토리(GUI 없는 환경)")
    parser.add_argument("--cols", type=int, default=9, help="체스보드 내부 코너 열 수")
    parser.add_argument("--rows", type=int, default=6, help="체스보드 내부 코너 행 수")
    parser.add_argument("--square-size", type=float, default=0.025, help="체스보드 한 칸 크기(m)")
    parser.add_argument("--out", type=str, default="camera_calib.npz", help="저장할 .npz 경로")
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)

    if args.images:
        paths = sorted(
            p for ext in ("*.jpg", "*.jpeg", "*.png")
            for p in glob.glob(os.path.join(args.images, ext))
        )
        if not paths:
            raise RuntimeError(f"이미지가 없습니다: {args.images}")
        K, dist, ret = calibrate_from_images(paths, pattern_size, args.square_size)
    else:
        K, dist, ret = calibrate_from_device(args.device, pattern_size, args.square_size)

    np.savez(args.out, K=K, dist=dist)
    print(f"저장 완료: {args.out} (재투영 오차={ret:.4f})")
    print("K=\n", K)
    print("dist=\n", dist)


if __name__ == "__main__":
    main()
