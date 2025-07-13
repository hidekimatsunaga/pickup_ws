# import serial
# import time

# ser = serial.Serial('/dev/serial/by-id/usb-Microchip_Technology_Inc._USB-RELAY1_X-RL2-if00',9600, timeout=3)

# serialCommand0 = "A0B0"
# serialCommand1 = "A1B1"

# #swtichON
# ser.write(serialCommand1.encode())
# time.sleep(1)

# #switchOFF
# # ser.write(serialCommand0.encode())

# ser.close()
import serial
import time
import keyboard

# --- 設定 ---
# ご自身の環境に合わせてシリアルポート名を変更してください
# Linux/Mac の例: '/dev/tty.usbserial-XXXX' や '/dev/serial/by-id/...'
# Windows の例: 'COM3'
SERIAL_PORT = '/dev/serial/by-id/usb-Microchip_Technology_Inc._USB-RELAY1_X-RL2-if00'
BAUD_RATE = 9600

# リレーに送信するコマンド
COMMAND_ON = "A1B1"
COMMAND_OFF = "A0B0"

# --- メイン処理 ---
ser = None  # シリアルオブジェクトを初期化
try:
    # シリアルポートに接続
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"シリアルポート {SERIAL_PORT} に接続しました。")
    print("-----------------------------------------")
    print("操作方法:")
    print("  'o' キー: リレーをONにします")
    print("  'f' キー: リレーをOFFにします")
    print("  'q' キー: プログラムを終了します")
    print("-----------------------------------------")

    while True:
        # 'o' キーが押されたらONコマンドを送信
        if keyboard.is_pressed('o'):
            ser.write(COMMAND_ON.encode())
            print("-> ONコマンドを送信しました")
            # 連続送信を防ぐために少し待機
            time.sleep(0.2)

        # 'f' キーが押されたらOFFコマンドを送信
        if keyboard.is_pressed('f'):
            ser.write(COMMAND_OFF.encode())
            print("-> OFFコマンドを送信しました")
            # 連続送信を防ぐために少し待機
            time.sleep(0.2)

        # 'q' キーが押されたらループを抜けて終了
        if keyboard.is_pressed('q'):
            print("終了します...")
            break

        # CPU使用率が高くなりすぎないように短いスリープを入れる
        time.sleep(0.05)

except serial.SerialException as e:
    # ポートが見つからない、または開けない場合のエラー処理
    print(f"エラー: シリアルポート {SERIAL_PORT} を開けませんでした。")
    print(f"詳細: {e}")
    print("ポート名が正しいか、デバイスが接続されているか確認してください。")

except ImportError:
    # keyboardライブラリがインストールされていない場合のエラー
    print("エラー: 'keyboard'ライブラリが見つかりません。")
    print("コマンドプロンプトやターミナルで 'pip install keyboard' を実行してインストールしてください。")

except Exception as e:
    # その他の予期せぬエラー
    print(f"予期せぬエラーが発生しました: {e}")

finally:
    # プログラム終了時に必ずシリアルポートを閉じる
    if ser and ser.is_open:
        ser.close()
        print(f"シリアルポート {SERIAL_PORT} を閉じました。")
