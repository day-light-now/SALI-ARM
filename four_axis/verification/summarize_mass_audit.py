"""Mechanical report from measured CAD audit; no model edits or motor control."""
from pathlib import Path
import json
from collections import defaultdict

folder=Path(__file__).parent/'gravity_preparation'
audit=json.loads((folder/'cad_mass_audit.json').read_text(encoding='utf-8'))
vendor=json.loads((folder/'motor_mass_sources.json').read_text(encoding='utf-8'))
parts=audit['parts']
def is_motor(p):
    return any(s in p['component'] for s in ['DAMIAO','DM-J4340'])
motors=defaultdict(list)
missing=defaultdict(list)
for p in parts:
    if is_motor(p):
        sections=p['component'].split('/')
        prefix='/'.join(sections[:2])
        motors[prefix].append(p)
    elif not p.get('material'):
        missing[p['document']].append(p)

lines=['# SLAI 质量与材料审计（2026-10-08）','',
       '结果仅供数据核对；未写入正式 URDF，未启动力矩控制。', '',
       f"读取 {len(parts)} 个叶级零件实例，其中 {sum(bool(p.get('material')) for p in parts)} 个有显式零件材料。",
       '读到铝合金 5052-H38 / 5052-H36 / 7075-T6、ABS 和普通碳钢。', '',
       '## 电机总重量来源','', '| 型号 | 官方整机质量 | 手册页 |', '|---|---:|---:|']
for model,p in vendor['motors'].items():
    lines.append(f"| {model} | {p['mass_kg']*1000:g} g | {p['source_page']} |")
lines+=['','手册仅给整机重量，不给转子/定子分拆重量、重心和惯量。',
        'CAD 电机叶件默认密度不能代替真实电机质量；完整电机也不能不加说明地全部计入某个运动连杆。','',
        '## 装配中的电机实例（仅作错误定位）','', '| CAD 实例 | CAD 叶件合计质量 |', '|---|---:|']
for name,rows in motors.items():
    lines.append(f"| {name} | {sum(p.get('mass_kg',0) for p in rows)*1000:.2f} g |")
lines+=['','## 未设置材料的非电机零件（按源文件去重）','',
        '这些不自动认定为错误：可能有实体材料/质量覆盖，但当前有效密度均需核查。','',
        '| 零件 | 所属连杆 | 数量 | CAD 有效密度 kg/m³ |','|---|---|---:|---:|']
for path,rows in missing.items():
    name=rows[0]['component'].split('/')[-1]
    links=', '.join(sorted({r['link'] for r in rows}))
    density=rows[0].get('effective_density_kg_m3')
    lines.append(f"| {name} | {links} | {len(rows)} | {density:.0f} |" if density else f"| {name} | {links} | {len(rows)} | 无有效体积 |")
lines+=['','轴承文件名称明确为安装包络示意，不应当直接把圆环体积乘默认密度当真实轴承重量。',
        '有一个 J4340 内部模型质量/体积为零，需区分参考曲面与缺失的实体。','',
        '## 当前 CAD 聚合质量（包含默认密度，不能作为最终参数）','',
        '| 连杆 | CAD 暂定质量 kg |','|---|---:|']
for name,p in audit['links'].items():
    lines.append(f"| {name} | {p.get('mass_kg',0):.6f} |")
lines+=['','## 下一步需要的确认','',
        '1. J3限位^link2 的材料是否漏设；打印支撑柱/隔离柱的实际材料和填充率。',
        '2. QJ203MA 轴承的实测或厂商质量；底座材料仅影响固定基座质量，不直接影响活动轴重力力矩。',
        '3. ABS 打印件暂为实心几何计算，实际填充率/称重不同会改变质量。',
        '4. 电机质量由助手核对资料；重心和转子/定子分拆需标记估算或测量，不能冒充官方数据。',
        '5. 确认当前 link4 上的 J4340 是否实际安装；以及底座安装方向、线缆和末端负载。','']
(folder/'质量材料审计.md').write_text('\n'.join(lines),encoding='utf-8')
print('\n'.join(lines))
