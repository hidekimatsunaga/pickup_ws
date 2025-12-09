#!/usr/bin/env python3
import re
from pathlib import Path

# ====== 設定ここから ======
# それぞれ自分の環境のパスに変えてね
STOP_HPP_PATH    = Path("/home/matsunaga-h/pickup_ws/src/hose_control/include/hose_control/motor_initial_position.hpp")
PICKUP_HPP_PATH  = Path("/home/matsunaga-h/pickup_ws/src/hose_control/include/hose_control/motor_pickup_position.hpp")
NARROW_HPP_PATH  = Path("/home/matsunaga-h/pickup_ws/src/hose_control/include/hose_control/narrow_space_controll_position.hpp")

# キャリブレーションノードの最後のログ
#   MOTOR_INIT_POS = [a0, a1, ..., a9]
# の中身をここにコピペしてね（先頭9つが糸、最後がモータ10）
NEW_MOTOR_INIT = [
    # 257.0, 265.0, 190.0, 91.0, 16.0, 15.0, 146.0, 103.0, 36.0, 154.0,
    280.37, 272.99, 232.03, 169.01, 56.60, 68.29, 318.43, 331.00, 106.08, 305.16,
]

# 小数点以下桁数
DIGITS = 2
# narrow_sequence の最後の列などで、絶対値がこれより大きい値は
# 「特殊なコマンド値」とみなしてオフセットを適用しない
SENTINEL_ABS_THRESHOLD = 10000.0
# ====== 設定ここまで ======


def parse_stop_hpp(path: Path):
    """stop_angles_ と stop_motor10_angle_ をパースして返す"""
    src = path.read_text(encoding="utf-8")

    # std::vector<float> stop_angles_ = { ... };
    m_vec = re.search(
        r"std::vector<float>\s+stop_angles_\s*=\s*\{([^}]*)\};",
        src, re.DOTALL
    )
    if not m_vec:
        raise RuntimeError("stop_angles_ definition not found")

    angles_str = m_vec.group(1)
    old_angles = [float(x.replace("f", "")) for x in angles_str.replace("\n", " ").split(",") if x.strip()]

    # float stop_motor10_angle_ = xxxf;
    m_10 = re.search(
        r"float\s+stop_motor10_angle_\s*=\s*([-\d\.]+)f?\s*;",
        src
    )
    if not m_10:
        raise RuntimeError("stop_motor10_angle_ definition not found")
    old_10 = float(m_10.group(1))

    return src, old_angles, old_10


def format_float(v: float, digits: int = 2, with_f: bool = True) -> str:
    fmt = f"{{:.{digits}f}}"
    s = fmt.format(v)
    return s + ("f" if with_f else "")


def update_stop_hpp(src: str, new_angles, new_10, digits: int = 2) -> str:
    """stop_angles_ と stop_motor10_angle_ 部分を書き換えた新しいソースを返す"""

    # stop_angles_
    new_angles_str = ", ".join(format_float(a, digits, True) for a in new_angles)
    src = re.sub(
        r"(std::vector<float>\s+stop_angles_\s*=\s*\{)[^}]*(\};)",
        r"\g<1>" + new_angles_str + r"\g<2>",
        src,
        flags=re.DOTALL
    )

    # stop_motor10_angle_
    new_10_str = format_float(new_10, digits, True)
    src = re.sub(
        r"(float\s+stop_motor10_angle_\s*=\s*)[-\d\.]+f?(\s*;)",
        r"\g<1>" + new_10_str + r"\g<2>",
        src
    )
    return src


def compute_offsets(old_angles, old_10, new_angles, new_10):
    if len(new_angles) != len(old_angles):
        raise RuntimeError("NEW_MOTOR_INIT の長さと stop_angles_ の長さが違う")

    # 9本 + motor10 の Δθ を作る
    offsets = [na - oa for na, oa in zip(new_angles, old_angles)]
    offsets.append(new_10 - old_10)
    return offsets


def update_sequence_hpp(path: Path, seq_name: str, offsets, digits: int = 2,
                        sentinel_abs_threshold: float | None = None):
    """
    inline const std::vector<std::vector<float>> seq_name = {
        { ... },
        { ... },
    };
    の部分を見つけて、各要素に offsets を加算して書き換える
    """
    src = path.read_text(encoding="utf-8")

    # シーケンス全体を抜き出す
    pattern = re.compile(
        rf"(inline\s+const\s+std::vector<std::vector<float>>\s+{seq_name}\s*=\s*\{{)(.*?)(\}};)",
        re.DOTALL
    )
    m = pattern.search(src)
    if not m:
        raise RuntimeError(f"{seq_name} definition not found in {path}")

    body = m.group(2)

    # 各行 { ... } をパース
    row_pattern = re.compile(r"\{([^{}]+)\}")
    rows = row_pattern.findall(body)
    if not rows:
        raise RuntimeError(f"No rows found in {seq_name} body")

    new_rows_strs = []
    for row_str in rows:
        # 1行分の float 群
        vals = [x.strip() for x in row_str.split(",") if x.strip()]
        floats = [float(v.replace("f", "")) for v in vals]

        if len(floats) != len(offsets):
            raise RuntimeError(
                f"Row length {len(floats)} != offsets length {len(offsets)} in {seq_name}"
            )

        new_floats = []
        for j, (v, off) in enumerate(zip(floats, offsets)):
            # sentinel 値（たとえば narrow_sequence の -11772 など）は触らない
            if sentinel_abs_threshold is not None and abs(v) > sentinel_abs_threshold:
                new_floats.append(v)
            else:
                new_floats.append(v + off)

        # 書き戻し
        formatted = ", ".join(format_float(v, digits, with_f=True) for v in new_floats)
        new_rows_strs.append("    {" + formatted + "}")

    new_body = ",\n".join(new_rows_strs) + "\n"

    # 置き換え
    new_src = pattern.sub(r"\1\n" + new_body + r"\3", src)

    # バックアップ & 上書き
    backup = path.with_suffix(path.suffix + ".bak")
    backup.write_text(src, encoding="utf-8")
    print(f"[{seq_name}] Backup created: {backup}")
    path.write_text(new_src, encoding="utf-8")
    print(f"[{seq_name}] Updated: {path}")


def main():
    if len(NEW_MOTOR_INIT) != 10:
        raise SystemExit("NEW_MOTOR_INIT は 10 要素（9本＋motor10）にしてください")

    # 1) stop_angles.hpp を読んで旧値取得
    stop_src, old_angles, old_10 = parse_stop_hpp(STOP_HPP_PATH)
    print("Old stop_angles_:", old_angles)
    print("Old stop_motor10_angle_:", old_10)

    new_angles = NEW_MOTOR_INIT[:9]
    new_10     = NEW_MOTOR_INIT[9]

    # 2) stop_angles.hpp を更新
    updated_stop_src = update_stop_hpp(stop_src, new_angles, new_10, digits=DIGITS)
    backup = STOP_HPP_PATH.with_suffix(STOP_HPP_PATH.suffix + ".bak")
    backup.write_text(stop_src, encoding="utf-8")
    print("[stop] Backup created:", backup)
    STOP_HPP_PATH.write_text(updated_stop_src, encoding="utf-8")
    print("[stop] Updated:", STOP_HPP_PATH)

    # 3) オフセットを計算
    offsets = compute_offsets(old_angles, old_10, new_angles, new_10)
    print("Offsets (Δθ0..8, Δθ10):", offsets)

    # 4) pickup_sequence を更新
    update_sequence_hpp(
        PICKUP_HPP_PATH,
        seq_name="pickup_sequence",
        offsets=offsets,
        digits=DIGITS,
        sentinel_abs_threshold=None,   # pickup は全部オフセット適用
    )

    # 5) narrow_sequence を更新
    update_sequence_hpp(
        NARROW_HPP_PATH,
        seq_name="narrow_sequence",
        offsets=offsets,
        digits=DIGITS,
        sentinel_abs_threshold=SENTINEL_ABS_THRESHOLD,  # -11772 などは無視
    )


if __name__ == "__main__":
    main()
