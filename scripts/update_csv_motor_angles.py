#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional


def parse_set_args(set_args: List[str]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    if not set_args:
        return result
    for item in set_args:
        if "=" not in item:
            raise argparse.ArgumentTypeError(f"--set の形式は motorX=value です: {item}")
        key, val = item.split("=", 1)
        key = key.strip()
        try:
            fval = float(val.strip())
        except ValueError:
            raise argparse.ArgumentTypeError(f"数値に変換できません: {item}")
        result[key] = fval
    return result


def parse_csv_floats(csv_text: str, expected_len: Optional[int] = None) -> List[float]:
    parts = [p.strip() for p in csv_text.split(',') if p.strip()]
    if expected_len is not None and len(parts) != expected_len:
        raise argparse.ArgumentTypeError(
            f"カンマ区切りの値は {expected_len} 個が必要です (got {len(parts)})."
        )
    vals: List[float] = []
    for p in parts:
        try:
            vals.append(float(p))
        except ValueError:
            raise argparse.ArgumentTypeError(f"数値に変換できません: {p}")
    return vals


def compute_offsets_from_first_row(
    first_row: Dict[str, str],
    columns: List[str],
    new_init: List[float] | None,
    set_targets: Dict[str, float],
) -> List[float]:
    old_vals: List[float] = []
    for c in columns:
        try:
            old_vals.append(float(first_row[c]))
        except Exception as e:
            raise RuntimeError(f"先頭行の列 {c} を数値化できません: {e}")

    # new_init が指定されていればそれを優先
    if new_init is not None:
        if len(new_init) != len(columns):
            raise RuntimeError(f"--new-init は {len(columns)} 個の値が必要です")
        return [nv - ov for nv, ov in zip(new_init, old_vals)]

    # --set のみが指定された場合、指定列は target に、未指定列は旧値据え置き
    target_vals: List[float] = []
    for c, ov in zip(columns, old_vals):
        if c in set_targets:
            target_vals.append(set_targets[c])
        else:
            target_vals.append(ov)
    return [tv - ov for tv, ov in zip(target_vals, old_vals)]


def apply_offsets_to_csv(
    in_path: Path,
    out_path: Path,
    columns: List[str],
    offsets_by_key: Dict[str | None, List[float]],
    digits: int | None,
    group_by: str | None,
) -> None:
    with in_path.open("r", newline="", encoding="utf-8") as f_in, out_path.open(
        "w", newline="", encoding="utf-8"
    ) as f_out:
        reader = csv.DictReader(f_in)
        fieldnames = reader.fieldnames
        if fieldnames is None:
            raise RuntimeError("CSV ヘッダが読み取れませんでした")

        # 指定列が存在するかチェック
        for c in columns:
            if c not in fieldnames:
                raise RuntimeError(f"CSV に列 {c} が見つかりません")
        if group_by is not None and group_by not in fieldnames:
            raise RuntimeError(f"CSV に group-by 列 {group_by} が見つかりません")

        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            key = row[group_by] if group_by is not None else None
            offsets = offsets_by_key.get(key)
            if offsets is None:
                # 未知キーはオフセット0扱い
                offsets = [0.0] * len(columns)
            new_row = dict(row)
            for c, off in zip(columns, offsets):
                try:
                    v = float(row[c])
                except Exception:
                    continue
                nv = v + off
                if digits is not None:
                    nv = round(nv, digits)
                new_row[c] = f"{nv}"
            writer.writerow(new_row)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "CSV のモータ角列に一括オフセットを適用します。"
            "\n- 先頭行を基準に --new-init（または --new-init-csv）や --set で新しい初期値を指定すると、"
            "全行に (new - old) の差分を加算して整列します。"
        )
    )
    parser.add_argument("csv", type=Path, help="入力CSVファイルのパス")
    parser.add_argument(
        "--columns",
        nargs="+",
        default=[f"motor{i}" for i in range(1, 11)],
        help="補正対象の列名（デフォルト: motor1..motor10）",
    )
    parser.add_argument(
        "--new-init",
        nargs="*",
        type=float,
        help=(
            "新しい初期値（列の数と同数の値）。指定すると先頭行値との差分を全行に加算。"
        ),
    )
    parser.add_argument(
        "--new-init-csv",
        type=str,
        help=(
            "カンマ区切りで初期値を指定（例: 328.97,318.60,...,160）。"
            "スペースを含んでいても1引数で受け取れる。"
        ),
    )
    parser.add_argument(
        "--set",
        action="append",
        help=(
            "個別列の目標初期値を motorX=value 形式で指定。"
            "複数回指定可。未指定列は旧値据え置きとして差分0。"
        ),
    )
    parser.add_argument(
        "--digits", type=int, default=None, help="丸め小数桁（未指定なら丸めなし）"
    )
    parser.add_argument(
        "--inplace",
        action="store_true",
        help="上書き保存（バックアップ .bak を作成）",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="差分のみ表示してファイルは書き換えない"
    )
    parser.add_argument(
        "--group-by",
        type=str,
        default=None,
        help="グルーピング列名（例: marker_id）。グループの先頭行を基準にオフセット計算",
    )
    parser.add_argument(
        "--motor10-mode",
        choices=["keep", "group-zero", "global-delta"],
        default="keep",
        help=(
            "motor10 の補正方式: keep=変更しない, "
            "group-zero=グループ先頭行が0になるよう一定値を加算, "
            "global-delta=全行に同じΔを加算"
        ),
    )
    parser.add_argument(
        "--motor10-delta",
        type=float,
        default=None,
        help="motor10 に全行で加算する一定Δ（--motor10-mode=global-delta 用）",
    )

    args = parser.parse_args()

    in_path: Path = args.csv
    if not in_path.exists():
        raise SystemExit(f"入力ファイルが見つかりません: {in_path}")

    # 全行を取得
    with in_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows:
            raise SystemExit("CSV が空です")
        fieldnames = reader.fieldnames or []
        # バリデーション
        for c in args.columns:
            if c not in fieldnames:
                raise SystemExit(f"CSV に列 {c} が見つかりません")
        if args.group_by is not None and args.group_by not in fieldnames:
            raise SystemExit(f"CSV に group-by 列 {args.group_by} が見つかりません")

    set_targets = parse_set_args(args.set)

    # --new-init があれば最優先。なければ --new-init-csv を使う。
    if args.new_init:
        new_init_vals: Optional[List[float]] = args.new_init
    elif args.new_init_csv:
        try:
            new_init_vals = parse_csv_floats(args.new_init_csv, expected_len=len(args.columns))
        except argparse.ArgumentTypeError as e:
            raise SystemExit(str(e))
    else:
        new_init_vals = None

    # グループごとに先頭行を基準にオフセット計算
    offsets_by_key: Dict[str | None, List[float]] = {}
    first_row_by_key: Dict[str | None, Dict[str, str]] = {}
    if args.group_by is None:
        first_row = rows[0]
        first_row_by_key[None] = first_row
        offsets_by_key[None] = compute_offsets_from_first_row(
            first_row=first_row,
            columns=args.columns,
            new_init=new_init_vals,
            set_targets=set_targets,
        )
    else:
        seen: set[str] = set()
        for r in rows:
            key = r[args.group_by]
            if key in seen:
                continue
            seen.add(key)
            first_row_by_key[key] = r
            offsets_by_key[key] = compute_offsets_from_first_row(
                first_row=r,
                columns=args.columns,
                new_init=new_init_vals,
                set_targets=set_targets,
            )

    # motor10 の特殊モードを適用
    if "motor10" in args.columns:
        idx10 = args.columns.index("motor10")
        if args.motor10_mode == "group-zero":
            # 各グループの先頭 motor10 が 0 になるように定数オフセット
            for key, first in first_row_by_key.items():
                try:
                    v0 = float(first["motor10"])
                except Exception:
                    v0 = 0.0
                offs = offsets_by_key.get(key)
                if offs is None:
                    offs = [0.0] * len(args.columns)
                    offsets_by_key[key] = offs
                offs[idx10] = offs[idx10] + (-v0)
        elif args.motor10_mode == "global-delta":
            # 全行に同じΔを加算
            if args.motor10_delta is not None:
                delta10 = args.motor10_delta
            else:
                # --set motor10=TARGET があれば CSV全体の先頭行からΔを計算
                target = None
                if "motor10" in set_targets:
                    target = set_targets["motor10"]
                if target is None:
                    raise SystemExit(
                        "--motor10-mode=global-delta には --motor10-delta か --set motor10=TARGET の指定が必要です"
                    )
                try:
                    first_global = float(rows[0]["motor10"])  # CSV最初の行
                except Exception:
                    first_global = 0.0
                delta10 = target - first_global

            for key, offs in offsets_by_key.items():
                offs[idx10] = offs[idx10] + delta10

    # ドライラン: オフセット内容を表示して終了
    if args.dry_run:
        print("[DRY-RUN] columns:", args.columns)
        if args.group_by is None:
            print("[DRY-RUN] group: (all)")
            print("[DRY-RUN] offsets:", offsets_by_key.get(None, []))
        else:
            print(f"[DRY-RUN] group-by: {args.group_by}")
            for k, offs in offsets_by_key.items():
                print(f"  key={k} -> offsets={offs}")
        return

    if args.inplace:
        backup = in_path.with_suffix(in_path.suffix + ".bak")
        backup.write_text(in_path.read_text(encoding="utf-8"), encoding="utf-8")
        out_path = in_path
        print(f"Backup created: {backup}")
    else:
        out_path = in_path.with_name(in_path.stem + "_updated" + in_path.suffix)

    apply_offsets_to_csv(
        in_path=in_path,
        out_path=out_path,
        columns=args.columns,
        offsets_by_key=offsets_by_key,
        digits=args.digits,
        group_by=args.group_by,
    )

    print(f"Updated: {out_path}")


if __name__ == "__main__":
    main()
