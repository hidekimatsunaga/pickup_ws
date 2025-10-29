# ...existing code...
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI/引数なしで保存済み動画の先頭をまとめて切るスクリプト。
設定は下の CONFIG ブロックだけを編集してください（引数不要）。
実行: python3 scripts/movie_cut.py
"""
import os
import glob
import subprocess
from datetime import datetime

# ===== CONFIG (ここだけ編集) =====
VIDEO_DIR = os.path.expanduser("~/pickup_ws/videos")   # 保存動画フォルダ
EXT = "*.mp4"                                          # 対象拡張子パターン
TRIM_MINUTES = 2.5                                     # 先頭から何分切るか（分単位）
# 指定するとそのファイル（またはワイルドカード）だけ処理する。
# 例:
#   VIDEO_FILE = None                 # 指定なし（既存の挙動）
#   VIDEO_FILE = "camera_2025*.mp4"   # ワイルドカード（VIDEO_DIR内で検索）
#   VIDEO_FILE = "/full/path/to/file.mp4"  # 絶対パス指定
VIDEO_FILE = "camera_20251029_075148.mp4"
PROCESS_ALL = True                                     # True: 全ファイル処理 / False: 最新ファイルのみ
OVERWRITE = False                                      # True: 元ファイルを上書き（注意） / False: 新規ファイル出力
DRY_RUN = False                                        # True: 実行コマンドのみ表示して実行しない
FFMPEG_BIN = "/usr/bin/ffmpeg"                         # ffmpeg 実行ファイルパス（必要ならフルパスにする）
# ===== CONFIG END =====

def find_targets():
    # VIDEO_FILE 指定があれば、それを優先して検索（ワイルドカード、相対/絶対対応）
    if VIDEO_FILE:
        vf = os.path.expanduser(VIDEO_FILE)
        # 絶対パスまたはワイルドカードを直接試す
        matches = sorted(glob.glob(vf))
        # 結果が無ければ VIDEO_DIR 下で解釈してみる
        if not matches:
            matches = sorted(glob.glob(os.path.join(VIDEO_DIR, vf)))
        # 最後に、もし拡張子省略などで単一ファイル名だったら直接結合して存在確認
        if not matches:
            candidate = os.path.join(VIDEO_DIR, vf)
            if os.path.exists(candidate):
                matches = [candidate]
        return matches
    # VIDEO_FILE 未指定の既存挙動
    p = os.path.join(VIDEO_DIR, EXT)
    files = sorted(glob.glob(p))
    if not files:
        return []
    if PROCESS_ALL:
        return files
    return [files[-1]]  # 最新1個

def build_out_path(inpath):
    base, ext = os.path.splitext(inpath)
    # TRIM_MINUTES が小数ならファイル名で小数点は p に置換しておく
    minutes_str = str(TRIM_MINUTES).replace('.', 'p')
    suffix = f"_cut{minutes_str}m"
    out = f"{base}{suffix}{ext}"
    if OVERWRITE:
        # overwrite: write to tmp then replace
        out = f"{base}{suffix}{ext}"
    return out

def run_ffmpeg_trim(infile, outfile, trim_s):
    # 推奨コマンド: -ss を入力の前に置くと高速シーク（ただし厳密なフレーム境界は保証されない）
    # 精度重視なら -ss を -i 後に置く（少し遅い）
    cmd = [FFMPEG_BIN, "-y", "-ss", f"{trim_s}", "-i", infile, "-c", "copy", outfile]
    return cmd

def main():
    trim_s = TRIM_MINUTES * 60
    targets = find_targets()
    if not targets:
        print("対象ファイルが見つかりません:", VIDEO_FILE if VIDEO_FILE else os.path.join(VIDEO_DIR, EXT))
        return
    print(f"処理対象 {len(targets)} ファイル, TRIM={TRIM_MINUTES} min ({trim_s}s), DRY_RUN={DRY_RUN}")
    for f in targets:
        out = build_out_path(f)
        if os.path.exists(out):
            print("出力ファイル既に存在 -> スキップ:", out)
            continue
        cmd = run_ffmpeg_trim(f, out, trim_s)
        print("CMD:", " ".join(cmd))
        if DRY_RUN:
            continue
        try:
            ret = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except subprocess.CalledProcessError as e:
            print(f"ffmpeg 失敗: {f}\nstderr:\n{e.stderr.decode(errors='ignore')}")
            if os.path.exists(out):
                try:
                    os.remove(out)
                except:
                    pass
            continue
        # overwrite モードなら元を差し替える（リネームでバックアップ）
        if OVERWRITE:
            bak = f + ".bak." + datetime.now().strftime("%Y%m%d_%H%M%S")
            try:
                os.replace(f, bak)
                os.replace(out, f)
                print(f"上書き完了: {f} (bak: {bak})")
            except Exception as e:
                print("上書き時エラー:", e)
        else:
            print("作成:", out)

if __name__ == "__main__":
    main()
# ...existing code...