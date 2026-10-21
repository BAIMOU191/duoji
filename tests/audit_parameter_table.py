"""Audit/update the current HTML against compiled firmware; no external reference or packages."""
from pathlib import Path
import argparse
import html
import json
import re
import subprocess
from verify_alignment import ROOT, OUT, calibrated, test_profile

DOC = ROOT/'参数总表.html'
REVIEW = OUT/'table_review'
BASE_PROJECT_NAMES = ('CH32V00X_APP_ADC_15kg','CH32V00X_APP_ADC_20kg','CH32V00X_APP_ADC_30kg')

def plain(s):
    return ' '.join(html.unescape(re.sub(r'<[^>]+>', ' ', s)).split())

def tables_of(s):
    return re.findall(r'<table\b[^>]*>.*?</table>', s, re.S)

def rows_of(table):
    return [[plain(c) for c in re.findall(r'<td\b[^>]*>(.*?)</td>', row, re.S)]
            for row in re.findall(r'<tr\b[^>]*>.*?</tr>', table, re.S) if '<td' in row]

def keyed(table):
    return {r[1]:r for r in rows_of(table)}

def row_html(cells, address=False):
    values = [html.escape(str(c)) for c in cells]
    if address:
        values[1] = '<code>'+values[1]+'</code>'
    return '    <tr>\n' + ''.join('     <td>'+v+'</td>\n' for v in values) + '    </tr>\n'

def replace_body(table, rows, address=False):
    body = '<tbody>\n'+''.join(row_html(r, address) for r in rows)+'   </tbody>'
    assert re.search(r'<tbody>.*?</tbody>', table, re.S)
    return re.sub(r'<tbody>.*?</tbody>', lambda _:body, table, flags=re.S)

def probe(project):
    synthetic = not calibrated(project)
    profile = test_profile(project) if synthetic else project
    exe = REVIEW/(project.name+'_schema.exe')
    inc = ['-I'+str(ROOT/'tests/stubs')]+['-I'+str(profile/d) for d in
            ['Application','Common','Drivers','System','SRC/Debug']]
    args = [r'D:\SoftWare\mingw64\bin\gcc.exe','-std=gnu99','-O0','-Wall','-Wextra','-Werror',
            *inc,ROOT/'tests/parameter_schema_probe.c','-lm','-o',exe]
    result = subprocess.run([str(x) for x in args], capture_output=True, text=True)
    (REVIEW/(project.name+'_schema_compile.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode:
        raise RuntimeError(result.stdout+result.stderr)
    schema = json.loads(subprocess.check_output([str(exe)],text=True))
    schema['test_model_only'] = synthetic
    schema['project'] = project.name
    config=(project/'Application/A_Config.h').read_text(encoding='utf-8')
    schema['default_formulas']={}
    for addr,field in [('30','VMAX_DPS'),('31','AMAX_DPS2'),('32','DMAX_DPS2')]:
        expression=re.search(r'^#define CFG_DEF_'+field+r'\s+([^\n]+)',config,re.M)[1]
        ratio=re.search(r'\*\s*(\d+)U?\s*/\s*(\d+)U?',expression)
        assert ratio, (project.name,field,expression)
        schema['default_formulas'][addr]={'numerator':int(ratio[1]),'denominator':int(ratio[2])}
    (REVIEW/(project.name+'_schema.json')).write_text(json.dumps(schema,indent=2),encoding='utf-8')
    return schema

def update(s, schema):
    tables = tables_of(s)
    assert len(tables)==15, 'Document table layout changed; review selectors before updating'
    flash, ram = keyed(tables[10]), keyed(tables[12])
    params = {f'{p["id"]:02X}':p for p in schema['params']}
    flash = {addr:row for addr,row in flash.items() if addr in params}
    ram = {addr:row for addr,row in ram.items() if addr in params or addr in schema['actions']}
    c = schema['constants']
    # Update all fixed factory values from Config_FillDefaults, not duplicated literals.
    for addr,row in flash.items():
        row[3] = str(params[addr]['size'])
        if re.fullmatch(r'\d+',row[4]):row[4] = str(params[addr]['default'])
    boot_ms=c['SERVO_BOOT_MOVE_MS']
    flash['23'][6]='1按'+str(boot_ms)+'ms请求转到保存位置，2原地保持，3自由释放'
    flash['23'][8]='位置模式上电复位的规划请求时间统一'+str(boot_ms)+'ms（'+f'{boot_ms/1000:g}'+'s），包括首帧PWM上电目标和反馈恢复后补执行的上电动作；后续普通运动指令按各自T执行。实际完成时间仍受运动能力约束。'
    flash['25'][8]='仅上电动作1使用；位置模式以'+str(boot_ms)+'ms作为规划请求时间，目标夹进SMI/SMX脉宽边界。'
    flash['35'][2] = '保持比例增益 P'
    flash['35'][6:] = ['0～'+str(c['CFG_HOLD_KP_MAX']),'‰/°',
        f'到位后，对超出到位窗的位移每1°输出标定电压的P‰；默认{c["CFG_DEF_HOLD_KP"]}即每超出1°输出{c["CFG_DEF_HOLD_KP"]/10:g}%标定电压。运行中每10ms最多改变1‰/°。0仅关闭保持P，不清除继承的托举积分，也不关闭I。该值表示相对出力增益，不是测得扭矩。']
    flash['37'][6:] = ['floor(硬件页08返回值×3/2)～500','0.01°',
        '默认50=0.50°，同时是到位窗和保持窗的半宽（只有这一个窗口，不再有齿隙和回差），下限=floor(1.5×最小可靠位移)。参考已停，误差在到位窗内且速度不超过噪声底时，捕获计数每拍加1，否则减2，达到20拍判到位；到位后误差超出该窗才施加P/I回正，以目标为中心。']
    flash['38'] = [0,'38','保持积分系数 I',2,c['CFG_DEF_HOLD_KI'],'读写',
                   '0～'+str(c['CFG_HOLD_KI_MAX']),'‰/(°·s)',
                   '对超出到位窗的位移每1°持续1s，输出增加标定电压的I‰；0停止新增保持积分，仍保留到位时继承的托举力。系数不再使用毫秒或1/s²单位，当前范围0～'+str(c['CFG_HOLD_KI_MAX'])+'；输出及积分仍受电压限幅约束。']
    flash['41'][8] = '过流判断每10ms一次，连续有效电流严格大于阈值时累计；持续时间向上取整到10ms。触发后卸力并冷却500ms，再清过流故障。'
    flash['42'][8] = '500表示50.0℃；每250ms采样，温度大于或等于阈值时累计，降至阈值以下5.0℃才清除过温故障。'
    flash['43'][8] = '持续时间向上取整为250ms采样次数；传感器读失败不当作降温，也不清除已有故障。'
    flash['44'][8] = '每20ms检测：实际位移≤阈值，且参考位移≥4×阈值或本周期饱和冻结≥15拍，才累计堵转；只有有效位置轨迹参与。'
    # Reflect the current writable SRAM aliases.
    for rt,fl in [('85','30'),('86','31'),('87','32'),('88','33'),('89','34'),('8A','35'),
                  ('8E','37'),('8F','38')]:
        f = flash[fl]
        ram[rt] = [0,rt,'实时'+f[2],f[3],'Flash '+fl,'读写',f[6],f[7],str(f[8])+'；仅改SRAM，不保存。']
    ram['80'][6] = '位置2字节＋时间/圈数2字节；各字段0～65535，位置另限500～2500'
    ram['80'][8] = '位置须在SMI/SMX内；位置模式值为ms，0最快；模式7/8值为圈数，9/10为秒，0无限。电机模式1500是零驱动，但计时/圈数任务不因此清除，C0也未必变空闲；终止任务请用81=3/DST。受理00，目标越界03，状态不允许04。'
    ram['82'][8] = '配置层初值1；上电自由释放、保护等可将其改为0。恢复默认会把此参数重置1，但不自动解除原有释放状态，实际状态以C0为准；可用ULR或新运动恢复。恢复只锚定当前位置，不恢复旧目标。写非法模式或受保护而不能恢复正常时返回04。'
    newtables = list(tables)
    for index,data in [(10,flash),(12,ram)]:
        rows=[]
        for number,addr in enumerate(sorted(data,key=lambda x:int(x,16)),1):
            row=data[addr];row[0]=number;rows.append(row)
        newtables[index]=replace_body(tables[index],rows,True)
    device=keyed(tables[7])
    for addr,row in device.items():row[3]=schema['reads'][addr]
    versions=[c['SERVO_VERSION_'+v] for v in ['MAJOR','MINOR','PATCH']]
    device['10'][4] = ' '.join(f'{v:02X}' for v in versions)+'（'+'.'.join(map(str,versions))+('，未发布）' if not any(versions) else '）')
    newtables[7]=replace_body(tables[7],device.values(),True)
    status=keyed(tables[13])
    status['C2'][7]='位置模式查询时调用位置读取并刷新有效位置；读取失败保留最近一次有效值。电机模式统一返回1500。C7则只使用缓存，不触发新采样。'
    status['C3'][7]='直接读取温度传感器，返回0.1℃；失败返回-32768。与C7的250ms保护任务缓存更新时刻不同。'
    status['C4'][7]='直接读取最近一次ADC电压缓存，例如700表示7.00V；C7使用10ms保护任务缓存。'
    status['C5'][7]='10ms保护任务缓存的电流大小，本地址不附有效标志；需结合C7的扭矩FFFF判断采样有效性，不能把无效读数当作真实绕组电流。'
    status['C6'][7]='下一段有效位置轨迹中，检测周期内实际位移大于堵转阈值后清除；关闭堵转保护也会清除，单纯发送新指令不保证清除。'
    newtables[13]=replace_body(tables[13],status.values(),True)
    commands=rows_of(tables[8])
    for row in commands:
        if row[0]=='位置/速度运动':
            row[2]='位置模式目标0500～2500，T为0000～9999ms，0按最快允许轨迹。时间还受本机V/A/D、平滑和最短运动时间约束；做不到指定时间时会延长，输出饱和也可能冻结轨迹时钟。电机模式前4位为开环驱动大小及方向，1500为零驱动；7/8的T为圈数，9/10为秒，T=0无限。'
        elif row[0]=='多圈运动':
            row[2]='仅磁编码IIC的位置模式支持此明文格式，二进制80没有N字段。目标0500～2500且在SMI/SMX内，N=0～9；按当前位置到目标的坐标差选择方向，差为0时取正方向，额外转N圈。T按总位移比例换算所需时间（每圈ms），仍受运动能力与最短时间约束；电位器版本忽略。'
        elif row[0]=='中位校准':
            if '电位器版会围绕校准锚点' not in row[2]:
                row[2]+='电位器版会围绕校准锚点按可测行程收缩有效角度，不保证仍有标称180°/270°。'
        elif row[0]=='零点校准':
            if '电位器版会按锚点' not in row[2]:
                row[2]+='电位器版会按锚点及剩余可测行程收缩有效角度；不保证仍有标称角度。'
        elif row[0]=='设置上电动作':
            row[2]='保存下次上电使用的动作：1在位置模式下以'+str(boot_ms)+'ms请求转到保存位置，2保持上电实际位置，3自由释放；检测到外部PWM时首帧位置目标优先，并使用同一上电请求时间。'
        elif row[0]=='保存上电位置':
            row[2]='在位置模式下将当前位置保存为上电目标；配合CSM1，下次上电按'+str(boot_ms)+'ms请求规划返回该位置，实际用时受运动能力约束。'
    newtables[8]=replace_body(tables[8],commands)
    modes=rows_of(tables[11])
    for row in modes:
        if row[0] in ['1','2','3','4']:
            row[2]=row[2].split('；角度为标称行程')[0]+'；角度为标称行程，电位器校准后可能收缩；方向还受该型号整机旋向配置影响。'
    newtables[11]=replace_body(tables[11],modes)
    # Keep historical columns, but regenerate their current column from current parameter rows.
    changes=rows_of(tables[14])
    for row in changes:
        match=re.fullmatch(r'(Flash|RAM) ([0-9A-F]{2})',row[0])
        if match:
            data=flash if match[1]=='Flash' else ram
            if match[2] in data:
                d=data[match[2]]
                row[2]=f'{d[2]}；{d[3]}字节；默认/来源{d[4]}；范围{d[6]}；单位{d[7]}'
            elif match[2] in ('36','8D'):
                row[2]='当前固件已移除此参数地址，不再提供保持齿隙；读取返回03，写入Flash 36返回03、RAM 8D返回06。保持窗口就是到位窗口，宽度由到位精度37/8E决定。'
            else:
                row[2]='当前固件已移除此参数地址，不再提供保持D；读取返回03，写入Flash 39返回03、RAM 90返回06。'
        if row[0]=='统一出厂默认值与未标定说明':
            row[2]='当前默认值：精度'+str(c['CFG_DEF_RESOLUTION_CDEG'])+'；保持P='+str(c['CFG_DEF_HOLD_KP'])+'（上限'+str(c['CFG_HOLD_KP_MAX'])+'）、I='+str(c['CFG_DEF_HOLD_KI'])+'；保持D与保持齿隙已移除，保持窗口=到位精度。电机型号参数仍须有效实测标定，禁止用测试模型替代生产标定。'
    newtables[14]=replace_body(tables[14],changes)
    index=iter(newtables)
    s=re.sub(r'<table\b[^>]*>.*?</table>',lambda _:next(index),s,flags=re.S)
    s=re.sub(r'(?<!二进制)广播限制','二进制广播限制',s)
    s=s.replace('超过温度阈值持续达到该时间后停机','温度≥阈值的有效采样累计达到向上取整后的持续时间才停机')
    s=s.replace('保持精度、齿隙和积分参数在控制拍生效，保持位置环增益缓慢跟随。',
                '到位精度和积分系数I在位置控制拍生效；保持P每10ms最多变化1/s。')
    s=s.replace('上电从Flash复制参数；轨迹限制下一次规划生效；保持增益缓慢跟随。',
                '初始化时从配置复制运行参数，上电动作/保护可能随后改变扭矩模式；轨迹限制下一次规划生效，保持P缓慢跟随，I在控制拍生效。')
    s=s.replace('默认6.0V；因0.2V间隔约束','默认6.0V；因0.2V间隔约束')
    s=s.replace('保持精度、齿隙、积分系数I和阻尼D在位置控制拍生效；保持P每10ms最多变化1/s。','到位精度和积分系数I在位置控制拍生效；保持P每10ms最多变化1/s。')
    s=s.replace('I/D在控制拍生效','I在控制拍生效')
    s=s.replace('36/37为齿隙/到位精度；38/8F为2字节积分系数I（1/s²，0～200），不再是ms；保持D及39/90地址已移除。','37为到位精度（保持窗口也用它）；38/8F为2字节积分系数I（1/s²，0～200），不再是ms；保持D、保持齿隙及39/90、36/8D地址已移除。')
    # Correct the example that previously implied a RAM override returns to factory values.
    s=s.replace('RAM地址86写入0，表示使用硬件信息中的最大加速度；掉电后恢复默认',
                'RAM地址86写入0，表示使用本机额定最大加速度；重启后从Flash 31恢复，而非强制回工厂默认值')
    s=s.replace('新增36/37/38与8D/8E/8F。',
                '37为到位精度（保持窗口也用它）；38/8F为2字节积分系数I（1/s²，0～200），不再是ms；保持D、保持齿隙及39/90、36/8D地址已移除。')
    s=s.replace('保持P每10ms最多变化1/s','保持P每10ms最多变化1‰/°')
    s=re.sub(r'35/8A改为2字节1/s，37为到位精度（保持窗口也用它）；38/8F为2字节积分系数I（1/s²，0～200），不再是ms；',
             '35/8A为2字节保持P（‰/°，0～'+str(c['CFG_HOLD_KP_MAX'])+'）；37为到位精度（保持窗口也用它）；38/8F为2字节积分系数I（‰/(°·s)，0～'+str(c['CFG_HOLD_KI_MAX'])+'），不再是ms或1/s²；',s)
    note=f'''<div class="note" id="hold-controller-rules"><strong>当前保持控制参数：</strong>
P：35/8A，默认{c['CFG_DEF_HOLD_KP']}，范围0～{c['CFG_HOLD_KP_MAX']}，单位‰/°；I：38/8F，默认{c['CFG_DEF_HOLD_KI']}，范围0～{c['CFG_HOLD_KI_MAX']}，单位‰/(°·s)。
‰指本机标定电压的千分之一：超出到位窗1°时，P={c['CFG_DEF_HOLD_KP']}输出{c['CFG_DEF_HOLD_KP']/10:g}%标定电压；I=10时该误差每持续1s再增加1%标定电压。输出受电压与积分限幅约束；相同数值表示相同标定电压比例，不代表测得扭矩相同。
保持D、保持齿隙已移除，39/90、36/8D不再是有效参数。到位精度37/8E默认{c['CFG_DEF_RESOLUTION_CDEG']}，单位0.01°。
只有一个窗口：到位窗=保持窗=到位精度（下限floor(1.5×最小可靠位移)），以目标为中心，不再叠加齿隙与回差。
窗口内保留到位时继承的托举积分，超出窗口的部分才施加P/I回正，回正力从窗口边沿连续长出（弹簧）；I=0停止新增保持积分，保留已有托举力。保持中不另加速度阻尼，靠电机反电势阻尼。
输出幅值小于动摩擦电压一半时置零驱动并短路制动；从保持转入新运动时，将保持P输出并入积分以衔接托举力。</div>'''
    pattern=r'<div class="note" id="hold-controller-rules">.*?</div>'
    if re.search(pattern,s,re.S):s=re.sub(pattern,lambda _:note,s,flags=re.S)
    else:
        anchor=re.search(r'<h2>\s*二、Flash',s).start()
        s=s[:anchor]+note+'\n  '+s[anchor:]
    limits='''<div class="note" id="business-frame-limits"><strong>业务协议长度与超时：</strong>
二进制请求LEN最大32字节，整帧最大40字节；明文整帧最多32字节。半帧连续10次串口任务未收到字节（正常调度约10ms）后丢弃，发送时应保持连续。
二进制广播只开放写80/81/82/83且不应答；明文255广播仍按各指令正常应答，多机同时回复可能冲突。
所有Flash设置均在后台保存，OK表示命令受理，不是落盘完成或运动完成确认。用户恢复默认重置参数，但不会自动解除原有释放或保护状态；请读C0确认，必要时发送ULR或新的有效运动指令。</div>'''
    pattern=r'<div class="note" id="business-frame-limits">.*?</div>'
    if re.search(pattern,s,re.S):s=re.sub(pattern,lambda _:limits,s,flags=re.S)
    else:
        anchor=re.search(r'<h3>\s*3\. 命令总表',s).start()
        s=s[:anchor]+limits+'\n  '+s[anchor:]
    return s

def audit(s, schema):
    tables=tables_of(s)
    sets={i:keyed(tables[i]) for i in [7,10,12,13]}
    issues=[]
    params={f'{p["id"]:02X}':p for p in schema['params']}
    expected_flash={k for k,p in params.items() if p['flash']}
    expected_ram={k for k,p in params.items() if not p['flash']}|set(schema['actions'])
    for idx,expect in [(10,expected_flash),(12,expected_ram)]:
        if set(sets[idx])!=expect:issues.append(f'Table {idx} missing/extra addresses: {sorted(expect-set(sets[idx]))}/{sorted(set(sets[idx])-expect)}')
    documented_reads={}
    for idx in [7,10,12,13]:
        for addr,row in sets[idx].items():
            size=int(re.match(r'\d+',row[3]).group())
            if row[5 if idx in [10,12] else 4 if idx==13 else 2]!='只写':documented_reads[addr]=size
            if addr in params:
                p=params[addr]
                if size!=p['size']:issues.append(f'{addr}: length {size} != {p["size"]}')
                if idx==10 and re.fullmatch(r'\d+',str(row[4])) and int(row[4])!=p['default']:
                    issues.append(f'{addr}: default {row[4]} != {p["default"]}')
                if idx==10 and addr=='35' and row[4]=='按型号（见说明）':
                    project=schema.get('project','')
                    label=project.replace('CH32V00X_APP-IIC_', 'IIC ').replace('CH32V00X_APP_ADC_', 'ADC ')
                    if not re.search(re.escape(label+'='+str(p['default']))+r'(?!\d)',row[8]):
                        issues.append(f'{project}: missing or incorrect model-specific hold P default')
                if idx==10 and addr in schema.get('default_formulas',{}):
                    percentage=re.search(r'×(\d+(?:\.\d+)?)%',row[4])
                    if percentage:
                        ratio=schema['default_formulas'][addr]
                        if float(percentage[1])*ratio['denominator']!=100*ratio['numerator']:
                            issues.append(f'{addr}: documented default ratio differs from source macro')
                if idx in [10,12] and re.fullmatch(r'\d+～\d+',row[6]) and p['max']:
                    lo,hi=map(int,row[6].split('～'))
                    if (lo,hi)!=(p['min'],p['max']):issues.append(f'{addr}: range {lo}..{hi} != {p["min"]}..{p["max"]}')
    if documented_reads!=schema['reads']:
        issues.append('Documented readable addresses/lengths do not match Bin_ParamRead')
    for addr in ['35','8A','38','8F']:
        row=sets[10 if addr in ['35','38'] else 12].get(addr)
        unit='‰/°' if addr in ['35','8A'] else '‰/(°·s)'
        if not row or row[7]!=unit:issues.append(f'{addr}: obsolete hold gain units')
    for addr in ['36','8D','39','90','8C']:
        if addr in sets[10] or addr in sets[12]:issues.append(f'{addr}: removed parameter is still listed as valid')
    for addr in ['35','8A','38','8F']:
        if addr not in schema.get('hold_boundary_checks',[]):issues.append(f'{addr}: write boundary check missing')
    for addr,size in schema['actions'].items():
        if int(sets[12][addr][3])!=size:issues.append(f'{addr}: action write length mismatch')
    text_rows=rows_of(tables[8])
    all_commands=' '.join(r[1] for r in text_rows)
    for key,digits in schema['text']:
        pattern=r'#000P'+re.escape(key)+(r'(?:n+|\d{'+str(digits)+r'})' if digits else '')+'!'
        if not re.search(pattern,all_commands):issues.append(f'Missing text instruction: {key} / {digits} digits')
    # Check all complete binary frames in the current manual, excluding explicit historical examples.
    current=s.split('<section id="changes-from-previous">',1)[0]
    frame_count=0
    for match in re.finditer(r'\bAA\s+66(?:\s+[0-9A-F]{2}){6,}\b',plain(current)):
        data=bytes.fromhex(match[0]);pos=0
        while pos<len(data):
            if data[pos:pos+2]!=b'\xaa\x66' or pos+5>len(data):break
            n=data[pos+4];f=data[pos:pos+8+n]
            crc=0
            for v in f[2:-3]:
                crc^=v
                for _ in range(8):crc=((crc<<1)^(7 if crc&128 else 0))&255
            if len(f)!=8+n or f[-2:]!=b'\x66\xaa' or crc!=f[-3]:issues.append('Invalid binary example: '+f.hex(' '))
            frame_count+=1;pos+=8+n
    return issues,frame_count

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fix',action='store_true')
    parser.add_argument('--project', help='Audit one firmware project, e.g. CH32V00X_APP_ADC_30kg')
    args=parser.parse_args()
    REVIEW.mkdir(parents=True,exist_ok=True)
    projects = [ROOT/args.project] if args.project else [ROOT/name for name in BASE_PROJECT_NAMES]
    schemas={p.name:probe(p) for p in projects}
    base=schemas['CH32V00X_APP_ADC_30kg']
    for name,other in schemas.items():
        assert other['reads']==base['reads'],name+' read schema differs'
        assert [(p['id'],p['size'],p['flags']) for p in other['params']]==[(p['id'],p['size'],p['flags']) for p in base['params']],name
        for key in ['CFG_DEF_HOLD_KI','CFG_HOLD_KP_MAX','CFG_HOLD_KI_MAX','CFG_DEF_RESOLUTION_CDEG','SERVO_BOOT_MOVE_MS']:
            assert other['constants'][key]==base['constants'][key],(name,key)
        c=other['constants'];params={p['id']:p for p in other['params']}
        for addr,key in [(0x30,'MOTOR_RATED_VEL_DPS'),(0x31,'MOTOR_RATED_ACC_DPS2'),(0x32,'MOTOR_RATED_DEC_DPS2')]:
            ratio=other['default_formulas'][f'{addr:02X}']
            assert params[addr]['default']==c[key]*ratio['numerator']//ratio['denominator'],(name,addr)
        assert c['CFG_RESOLUTION_MIN_CDEG']==c['MOTOR_MIN_STEP_CDEG']*3//2
        assert c['PROT_STALL_THRESHOLD_MAX_CDEG']==c['MOTOR_RATED_VEL_DPS']//2
        assert c['CFG_SMOOTH_MIN_MS']==(max(c['MOTOR_RATED_ACC_DPS2'],c['MOTOR_RATED_DEC_DPS2'])+1999)//2000
    (REVIEW/'document_schema.json').write_text(json.dumps(base,indent=2),encoding='utf-8')
    s=DOC.read_text(encoding='utf-8');before,count=audit(s,base)
    for issue in before:print('FOUND:',issue,flush=True)
    if args.fix:
        from sync_parameter_table import sync
        s=sync(s,schemas);DOC.write_text(s,encoding='utf-8',newline='\n')
    after=[]
    for name,schema in schemas.items():
        issues,count=audit(s,schema)
        after.extend(f'{name}: {issue}' for issue in issues)
    report={'before':before,'after':after,'binary_examples_checked':count,'projects':list(schemas),
            'readable_addresses_checked':len(base['reads']),'ordinary_writable_parameters_checked':len(base['params'])}
    (REVIEW/'parameter_table_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for issue in after:print('MISMATCH:',issue)
    if after:raise SystemExit(1)
    print(f'PASS: {len(schemas)} project(s); {len(base["reads"])} readable addresses, {len(base["params"])} ordinary writable parameters and {count} binary examples',flush=True)

if __name__=='__main__':main()
