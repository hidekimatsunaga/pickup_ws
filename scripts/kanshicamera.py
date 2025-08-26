import cv2

# カメラのキャプチャを開始
# 0は通常、内蔵カメラまたは最初に認識されたUSBカメラを指します。
# 複数のカメラがある場合は、1, 2, ...と数字を変えて試してください。
cap = cv2.VideoCapture(6)

# カメラが正常に開かれたかを確認
if not cap.isOpened():
    print("エラー: カメラを開けませんでした。")
    exit()

# 無限ループでフレームを読み込み、表示し続ける
while True:
    # 1フレーム分の画像データを読み込む
    # retは読み込みが成功したかどうかのブール値 (True/False)
    # frameは読み込まれた画像データ (NumPy配列)
    ret, frame = cap.read()

    # フレームの読み込みに失敗した場合はループを抜ける
    if not ret:
        print("エラー: フレームを読み込めませんでした。")
        break

    # "Camera Feed"という名前のウィンドウにフレームを表示
    cv2.imshow("Camera Feed", frame)

    # 'q'キーが押されたらループを抜ける
    # cv2.waitKey(1) は1ミリ秒キー入力を待つ。& 0xFF はおまじない。
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 使い終わったら、キャプチャを解放し、ウィンドウをすべて閉じる
cap.release()
cv2.destroyAllWindows()