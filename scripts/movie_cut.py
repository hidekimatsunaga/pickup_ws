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
import json
from datetime import datetime

# ===== CONFIG (ここだけ編集) =====
VIDEO_DIR = os.path.expanduser("~/pickup_ws/videos")   # 保存動画フォルダ
EXT = "*.mp4"                                          # 対象拡張子パターン
TRIM_MINUTES = 0.25                                     # 先頭から何分切るか（分単位）
TRIM_END_MINUTES = 1                                   # 後半から何分切るか（分単位、0=切らない）
# 指定するとそのファイル（またはワイルドカード）だけ処理する。
# 例:
#   VIDEO_FILE = None                 # 指定なし（既存の挙動）
#   VIDEO_FILE = "camera_2025*.mp4"   # ワイルドカード（VIDEO_DIR内で検索）
#   VIDEO_FILE = "/full/path/to/file.mp4"  # 絶対パス指定
VIDEO_FILE = "camera_20251226_043718.mp4"
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
    # 後半削除を指定していれば、ファイル名に追記
    if TRIM_END_MINUTES > 0:
        end_minutes_str = str(TRIM_END_MINUTES).replace('.', 'p')
        suffix += f"_end{end_minutes_str}m"
    out = f"{base}{suffix}{ext}"
    if OVERWRITE:
        # overwrite: write to tmp then replace
        out = f"{base}{suffix}{ext}"
    return out

def get_video_duration(infile):
    """ffprobe を使って動画の長さを秒単位で取得する"""
    try:
        cmd = [
            "ffprobe", "-v", "error",
            "-show_entries", "format=duration",
            "-of", "json",
            infile
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        data = json.loads(result.stdout)
        duration = float(data['format']['duration'])
        return duration
    except Exception as e:
        print(f"動画の長さ取得エラー ({infile}): {e}")
        return None

def run_ffmpeg_trim(infile, outfile, trim_s, trim_end_s=0):
    # 後半のカットがある場合は、動画の長さから後半を引いた長さを計算する
    if trim_end_s > 0:
        duration = get_video_duration(infile)
        if duration is None:
            return None
        # 実際の終了時間 = 全体の長さ - 後半カット時間
        end_time = duration - trim_end_s
        if end_time <= trim_s:
            print(f"警告: 先頭カット({trim_s}s) + 後半カット({trim_end_s}s) = 全体削除になる恐れ ({infile})")
            return None
        # -ss (開始時刻) から -t (継続時間) でカット
        cmd = [FFMPEG_BIN, "-y", "-ss", f"{trim_s}", "-i", infile, "-t", f"{end_time - trim_s}", "-c", "copy", outfile]
    else:
        # 後半カットなし: 先頭から先へ
        cmd = [FFMPEG_BIN, "-y", "-ss", f"{trim_s}", "-i", infile, "-c", "copy", outfile]
    return cmd

def main():
    trim_s = TRIM_MINUTES * 60
    trim_end_s = TRIM_END_MINUTES * 60
    targets = find_targets()
    if not targets:
        print("対象ファイルが見つかりません:", VIDEO_FILE if VIDEO_FILE else os.path.join(VIDEO_DIR, EXT))
        return
    print(f"処理対象 {len(targets)} ファイル, TRIM_START={TRIM_MINUTES} min ({trim_s}s), TRIM_END={TRIM_END_MINUTES} min ({trim_end_s}s), DRY_RUN={DRY_RUN}")
    for f in targets:
        out = build_out_path(f)
        if os.path.exists(out):
            print("出力ファイル既に存在 -> スキップ:", out)
            continue
        cmd = run_ffmpeg_trim(f, out, trim_s, trim_end_s)
        if cmd is None:
            print(f"スキップ: {f}")
            continue
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