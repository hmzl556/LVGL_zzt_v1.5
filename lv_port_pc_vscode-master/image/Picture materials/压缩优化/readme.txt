yunxingbackground 压缩试转结果
================================

源图: ../yunxingbackground.png
色深: RGB565（与现网 yunxingbackground.c 一致）
屏尺寸参考: 1600x600（源图约 1601x601）

本目录仅存放试转产物，未自动替换 src/ui/image/yunxingbackground.c。

体积对比（约）
--------------
- 现用未压缩 .c 文本: ~11.0 MB（像素数据约 1.92 MB）
- RLE  .bin: ~0.96 MB
- RLE  .c  : ~5.13 MB（C 数组文本膨胀）
- LZ4  .bin: ~0.40 MB   ← 本次 Flash 体积最优
- LZ4  .c  : ~2.14 MB

建议
----
1. 满屏背景优先试 LZ4（本图 LZ4 明显小于 RLE）。
2. 若要编进固件：把 LZ4/yunxingbackground.c 拷到 src/ui/image/ 覆盖同名文件后重编。
3. 运行时需解压整图到 RAM（约 宽x高x2 ≈ 1.9 MB），进运行页会多占这块内存。
4. lv_conf.h 需保持: LV_USE_LZ4_INTERNAL=1（或 EXTERNAL）、LV_BIN_DECODER_RAM_LOAD=1。
5. 若用 .bin 文件方式：lv_image_set_src(img, "A:.../yunxingbackground.bin");

生成命令备忘
------------
python lvgl/scripts/LVGLImage.py --ofmt C --cf RGB565 --compress LZ4 -o <outdir> --name yunxingbackground <png>
python lvgl/scripts/LVGLImage.py --ofmt C --cf RGB565 --compress RLE -o <outdir> --name yunxingbackground <png>
