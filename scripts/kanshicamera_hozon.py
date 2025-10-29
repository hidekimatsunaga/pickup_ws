#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
from datetime import datetime

# 設定: 出力ディレクトリ・コーデック等
OUT_DIR = os.path.expanduser("~/pickup_ws/videos")
FOURCC = "mp4v"   # 'mp4v' (mp4) / 'XVID' (avi) など環境に合わせて変更
DEFAULT_FPS = 30.0

# --- iPhone12 相当の解像度設定 ---
# TARGET_RESOLUTION に以下のキーを指定:
#  - 'iphone12_1080p' -> 1920x1080 (横向き)
#  - 'iphone12_4k'    -> 3840x2160 (横向き, 高負荷)
TARGET_RESOLUTION = 'iphone12_1080p'
RESOLUTIONS = {
    'iphone12_1080p': (1920, 1080),
    'iphone12_4k':    (3840, 2160),
}
# ------------------------------------------------

os.makedirs(OUT_DIR, exist_ok=True)

# カメラのキャプチャを開始（必要ならインデックスを変えてください）
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("エラー: カメラを開けませんでした。")
    exit()

# 解像度をリクエスト（カメラ/ドライバがサポートしていれば反映される）
if TARGET_RESOLUTION in RESOLUTIONS:
    req_w, req_h = RESOLUTIONS[TARGET_RESOLUTION]
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(req_w))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(req_h))
else:
    req_w = req_h = None

# 実際のFPSを取得（取得できない場合は DEFAULT_FPS を使用）
fps = cap.get(cv2.CAP_PROP_FPS)
if fps is None or fps <= 0 or fps != fps:
    fps = DEFAULT_FPS

# 最初のフレームで解像度を決定
ret, frame = cap.read()
if not ret or frame is None:
    print("エラー: 初期フレームを読み込めませんでした。")
    cap.release()
    exit()

height, width = frame.shape[:2]
frame_size = (width, height)

# 実際に取得できた解像度を表示（リクエストと比較）
print(f"requested resolution: {req_w}x{req_h}" if req_w is not None else "no resolution requested")
print(f"actual capture resolution: {frame_size[0]}x{frame_size[1]}")
if req_w is not None and (frame_size[0] != req_w or frame_size[1] != req_h):
    print("注意: カメラ/ドライバがリクエスト解像度をサポートしていないため、異なるサイズになっています。")

# 動画保存用の状態
recording = False
writer = None
file_path = None

print("操作: 'r' 録画開始/停止, 'q' 終了")

# 既に読み込んだ最初のフレームを表示／処理ループに使う
while True:
    # 最初のフレームは既に取得済み、それ以降は通常に読む
    if frame is None:
        ret, frame = cap.read()
    else:
        ret = True

    if not ret or frame is None:
        print("フレーム読み込みエラーまたはストリーム終了")
        break

    # 録画中なら書き込み
    if recording and writer is not None:
        writer.write(frame)

    cv2.imshow("Camera Feed", frame)

    # キー処理（waitKeyはミリ秒）
    key = cv2.waitKey(1) & 0xFF
    if key == ord('r'):
        # 録画トグル
        if not recording:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            base = f"camera_{ts}"
            filename = f"{base}.mp4" if FOURCC.lower() == "mp4v" else f"{base}.avi"
            file_path = os.path.join(OUT_DIR, filename)
            fourcc = cv2.VideoWriter_fourcc(*FOURCC)
            writer = cv2.VideoWriter(file_path, fourcc, fps, frame_size)
            if not writer.isOpened():
                print("エラー: VideoWriter を開けませんでした。別の FOURCC を試してください。")
                writer = None
            else:
                recording = True
                print(f"録画開始: {file_path} (fps={fps:.1f}, size={frame_size})")
        else:
            # 停止
            recording = False
            if writer is not None:
                writer.release()
                writer = None
            print(f"録画停止: {file_path}")
    elif key == ord('q'):
        # 終了
        if recording and writer is not None:
            writer.release()
            writer = None
            print(f"録画停止: {file_path}")
        break

    # 次フレームを読み込みループへ
    ret, frame = cap.read()

# 後処理
cap.release()
cv2.destroyAllWindows()
if recording and writer is not None:
    writer.release()
    print(f"録画ファイル保存: {file_path}")