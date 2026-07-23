import os
import shutil

DIR = r"F:\A_ZZT_Project\Xiaoya_Xiyiji\LVGL_zzt_v1.2\lv_port_pc_vscode-master\image\AI_create_image"
# mapping from current Chinese names (after number_) to pinyin
mapping = {
    '快洗': 'kuaixi',
    '速洗': 'suxi',
    '羽绒服': 'yurongfu',
    '羊毛': 'yangmao',
    '冷水洗': 'lengshuixi',
    '大件': 'dajian',
    '单脱水': 'dantuoshu',
    '筒自洁': 'tongzijie',
    '漂脱': 'piaotuo',
    '节能': 'jieneng',
    '棉麻': 'mianma',
    '混合洗': 'hunhexi',
    'AI智洗': 'aizhixi'
}

files = sorted(os.listdir(DIR))
print('Found', len(files), 'files')
for fname in files:
    if not fname.lower().endswith('.png'):
        continue
    parts = fname.split('_', 1)
    if len(parts) != 2:
        print('Skipping unexpected filename', fname)
        continue
    num = parts[0]
    name = parts[1]
    # remove extension
    if name.lower().endswith('.png'):
        name = name[:-4]
    pinyin = mapping.get(name)
    if pinyin is None:
        print('No mapping for', name, '-> skip')
        continue
    newname = f"{num}_{pinyin}.png"
    oldpath = os.path.join(DIR, fname)
    newpath = os.path.join(DIR, newname)
    print('Renaming', fname, '->', newname)
    # overwrite if exists
    if os.path.exists(newpath):
        os.remove(newpath)
    os.rename(oldpath, newpath)
print('Done')
