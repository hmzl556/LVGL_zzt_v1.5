#!/usr/bin/env python3
"""Compare ui-商用洗.c Chinese text vs lv_font --symbols coverage."""
import re
import pathlib

root = pathlib.Path(__file__).resolve().parents[1]
ui_file = root / "src/ui/ui-商用洗.c"
fonts_dir = root / "src/ui/fonts"
out = root / "tools/font_coverage_report.txt"

font_symbols = {}
for f in fonts_dir.glob("ui_font_SC_*.c"):
    text = f.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"--symbols\s+(.+?)\s+--format", text, re.S)
    if m:
        font_symbols[f.stem] = set(m.group(1).strip())

ui_text = ui_file.read_text(encoding="utf-8", errors="replace")

zh_block = re.search(r"\[UI_LANG_ZH\]\s*=\s*\{(.*?)\n\s*\},\s*\n\s*\[UI_LANG_EN\]", ui_text, re.S)
zh_strings = []
if zh_block:
    for m in re.finditer(r'\[STR_[^\]]+\]\s*=\s*"([^"]*)"', zh_block.group(1)):
        key = m.group(0).split("[")[1].split("]")[0]
        zh_strings.append((key, m.group(1)))

lines = [f"zh_strings parsed: {len(zh_strings)}"]

def chars_in(s):
    out = set()
    for ch in s:
        if ord(ch) > 127 or ch in "-—/，,.•：℃￥¥？?（）！!₃、():。； \n":
            out.add(ch)
    return out

def fmt_chars(cs):
    return " ".join(f"{c}(U+{ord(c):04X})" for c in sorted(cs, key=ord))

all_ui_chars = set()
for _, s in zh_strings:
    all_ui_chars |= chars_in(s)
for s in ["门锁开", "门锁关", "—— —— ——", "启停", "电源"]:
    all_ui_chars |= chars_in(s)

focus = ["ui_font_SC_20", "ui_font_SC_30", "ui_font_SC_35", "ui_font_SC_50", "ui_font_SC_125"]
font_usage = {
    "ui_font_SC_20": "顶栏：童锁/启停/电源、4G/时间",
    "ui_font_SC_30": "报警页 E1-E15 标题/说明、管理员/支付/循环等主文案",
    "ui_font_SC_35": "主页底部、运行页阶段、自检页、支付金额",
    "ui_font_SC_50": "程序名、报警码 Ex、支付完成/自检型号/完成标题",
    "ui_font_SC_125": "运行页倒计时（仅数字与英文）",
}

lines.append("\n=== 各字号缺失字符（相对全部 UI 中文文案）===\n")
for fname in focus:
    syms = font_symbols.get(fname, set())
    missing = sorted(all_ui_chars - syms, key=ord)
    lines.append(f"{fname} — {font_usage[fname]}")
    lines.append(f"  缺失 {len(missing)} 个: {fmt_chars(missing) if missing else '(无)'}")

fault = [(k, s) for k, s in zh_strings if k.startswith("STR_ALARM_E")]
fault_chars = set()
for _, s in fault:
    fault_chars |= chars_in(s)

sc30 = font_symbols.get("ui_font_SC_30", set())
m30_fault = sorted(fault_chars - sc30, key=ord)
lines.append("\n=== E1-E15 故障文案 vs ui_font_SC_30（报警页 30 号字）===\n")
lines.append(f"缺失 {len(m30_fault)} 个: {fmt_chars(m30_fault) if m30_fault else '(无)'}")

if m30_fault:
    lines.append("\n逐条故障缺字:")
    miss_set = set(m30_fault)
    for k, s in fault:
        miss = sorted(set(c for c in s if c in miss_set), key=ord)
        if miss:
            lines.append(f"  {k}: {fmt_chars(miss)}")
            lines.append(f"    全文: {s.replace(chr(10), ' / ')}")

areas = {
    "报警 E1-E15 + 页脚": [s for k, s in zh_strings if k.startswith("STR_ALARM_")],
    "厂商维护/自检/循环": [s for k, s in zh_strings if k.startswith(("STR_VENDOR_", "STR_SELF_", "STR_CYCLE_"))],
    "管理员/设置": [s for k, s in zh_strings if not k.startswith(("STR_ALARM_", "STR_VENDOR_", "STR_SELF_", "STR_CYCLE_", "STR_PAY_", "STR_RUN_", "STR_END_", "STR_PROG_D"))],
    "支付/运行/结束/程序名": [s for k, s in zh_strings if k.startswith(("STR_PAY_", "STR_RUN_", "STR_END_", "STR_PROG_D"))],
}
lines.append("\n=== ui_font_SC_30 分区域缺字 ===")
for area, strs in areas.items():
    cs = set()
    for s in strs:
        cs |= chars_in(s)
    miss = sorted(cs - sc30, key=ord)
    lines.append(f"\n{area} — 缺 {len(miss)} 个:")
    lines.append(f"  {fmt_chars(miss) if miss else '(无)'}")

sc35 = font_symbols.get("ui_font_SC_35", set())
lines.append("\n=== ui_font_SC_35 缺字 ===")
for label, strs in [
    ("自检页硬编码", ["门锁开", "门锁关", "—— —— ——"]),
    ("运行/程序底部", [s for k, s in zh_strings if k.startswith(("STR_RUN_", "STR_PROG_D"))]),
]:
    cs = set()
    for s in strs:
        cs |= chars_in(s)
    miss = sorted(cs - sc35, key=ord)
    lines.append(f"\n{label} — 缺 {len(miss)} 个: {fmt_chars(miss) if miss else '(无)'}")

sc50 = font_symbols.get("ui_font_SC_50", set())
lines.append("\n=== ui_font_SC_50 缺字 ===")
for label, strs in [
    ("报警码 Ex", ["E1","E2","E3","E4","E5","E6","E7","E8","E9","E10","E11","E12","E13","E14","E15"]),
    ("程序名/完成标题", [s for k, s in zh_strings if k.startswith(("STR_PROG_D", "STR_END_", "STR_SELF_CHECK", "STR_PAY_DONE"))]),
]:
    cs = set()
    for s in strs:
        for ch in s:
            cs.add(ch)
    miss = sorted(cs - sc50, key=ord)
    lines.append(f"\n{label} — 缺 {len(miss)} 个: {fmt_chars(miss) if miss else '(无)'}")

sc20 = font_symbols.get("ui_font_SC_20", set())
miss20 = sorted(set("童锁启停电源") - sc20, key=ord)
lines.append("\n=== ui_font_SC_20 顶栏缺字 ===")
lines.append(f"缺 {len(miss20)} 个: {fmt_chars(miss20) if miss20 else '(无)'}")

all_miss30 = sorted(all_ui_chars - sc30, key=ord)
if all_miss30:
    lines.append("\n=== 含 ui_font_SC_30 缺字的 UI 字符串 ===")
    miss_set = set(all_miss30)
    for k, s in zh_strings:
        if any(c in miss_set for c in s):
            hit = sorted(set(c for c in s if c in miss_set), key=ord)
            lines.append(f"  {k}: 缺 {fmt_chars(hit)} | {s.replace(chr(10), ' / ')}")

out.write_text("\n".join(lines), encoding="utf-8")
print(f"Wrote {out}")
