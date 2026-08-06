图片批量压缩产物
================

源目录: ../picture/
输出目录: 本目录（压缩优化/）

转换规则
--------
| 源子目录                    | 色深        | 压缩 | 说明              |
|-----------------------------|-------------|------|-------------------|
| Monochrome_or_Few_Colors/   | RGB565A8    | LZ4  | 少色图标（含透明） |
| multicolour/                | RGB565A8    | LZ4  | 彩色+透明（见下例外） |
| program/                    | RGB565A8    | LZ4  | 程序图            |
| qr-codes/（仅 .png）        | RGB565      | LZ4  | 收款码            |

输出结构
--------
  Monochrome_or_Few_Colors/{c,bin}/…
  multicolour/{c,bin}/…
  program/{c,bin}/…          （保留相对子目录）
  qr-codes/{c,bin}/…         （保留相对子目录）

体积汇总（约）
--------------
- Monochrome_or_Few_Colors: 36 张  RGB565A8+LZ4（已替换；体积大于原 I1+RLE）
- multicolour:              19 张  bin≈0.61 MB   c≈3.28 MB
- program:                  15 张  bin≈0.74 MB   c≈3.98 MB
- qr-codes:                 20 张  bin≈0.52 MB   c≈2.84 MB
- 合计:                     90 张  bin≈1.89 MB   c≈10.2 MB

说明
----
1. .bin 体积远小于 .c（C 数组是十六进制文本膨胀）。固件内嵌用 .c；运行时从文件系统加载用 .bin。
2. 彩色类默认 RGB565A8 以保留透明。以下已改为纯 RGB565 + LZ4 并覆盖原产物：
   QR_code、fault_logo、yunxingbackground、admin_button_box、auto_put_btn_box、
   bar_01、input_box、time_set_box、title_bar。
3. 本目录未自动替换 src/ui/image/ 下现用资源。
4. 使用压缩图需: LV_USE_RLE / LV_USE_LZ4_INTERNAL、LV_BIN_DECODER_RAM_LOAD=1；运行时解压约占接近未压缩位图的 RAM。
5. 转换脚本: _batch_convert.py（I1 依赖 tools/pngquant/pngquant/pngquant.exe）。
6. 详细逐文件尺寸见 batch_convert_summary.txt（重跑单色后可能只含最近一次类别；以本 readme 汇总为准）。

历史试转
--------
根下若仍有 RLE/、LZ4/，为早期 yunxingbackground 单独试转产物，可忽略；正式结果以 multicolour/ 下对应文件为准。
