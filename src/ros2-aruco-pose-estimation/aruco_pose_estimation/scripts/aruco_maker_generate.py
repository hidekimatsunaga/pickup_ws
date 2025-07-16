import cv2
import cv2.aruco as aruco
import numpy as np

# 使用する辞書（launchで設定したものと合わせる：例 DICT_4X4_50）
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# ID番号（0〜49の中から任意）
marker_id = 2

# マーカーサイズ（ピクセル単位）
marker_size = 700  # 高解像度印刷したいなら大きめで

# マーカー画像を生成
marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# 保存
cv2.imwrite("aruco_marker_id2.png", marker_image)
