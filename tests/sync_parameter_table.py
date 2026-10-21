"""Maintain the user reference for ADC 15kg, 20kg and 30kg firmware."""
import re
import audit_parameter_table as a


def lighting_section():
    sources = [(a.ROOT/name/'Application/A_Tasks.c').read_text(encoding='utf-8')
               for name in a.BASE_PROJECT_NAMES]
    assert len(set(sources)) == 1, 'Review model-specific LED behavior before publishing'
    source = sources[0]
    patterns = re.search(r's_state_led_patterns\[\]\[4\]\s*=\s*\{(.*?)\n\};', source, re.S)[1]
    values = [tuple(map(int, re.findall(r'\d+(?=U)', row)))
              for row in re.findall(r'\{([^}]+)\}', patterns)]
    assert values == [(0,0,0,0),(255,80,0,0),(255,0,255,0),(255,0,0,25),(0,255,0,25),(0,0,255,0)]
    assert re.search(r'#define TASK_STATE_PERIOD_MS\s+20U', source)
    assert re.search(r'#define STATE_LED_STEP\s+5U', source)
    rows = [
        ['彩虹渐变', '红→黄→绿→青→蓝→紫→红，约6.12秒一轮',
         '正常工作：运动、到位保持或位置反馈恢复过程中。',
         '需要确认运动是否完成时，读取工作状态C0；彩虹灯本身不区分运动和保持。'],
        ['橙色常亮', '持续亮橙色', '过温保护已触发。',
         '读取C1确认故障，读取C3查看温度；待温度降至恢复条件后再运行。'],
        ['紫色常亮', '持续亮紫色', '过流保护已触发。',
         '检查负载和机构是否受阻，并读取C1、C5；待故障解除后再运行。'],
        ['红色慢闪', '亮0.5秒、灭0.5秒，约每秒一轮', '电源异常：欠压、过压或电压采样无效。',
         '检查供电和接线；读取C1区分原因，读取C4查看电压，并核对电压设置47/48。'],
        ['绿色慢闪', '亮0.5秒、灭0.5秒，约每秒一轮', '位置反馈故障或其他未归类的设备异常。',
         '读取C0、C1，并检查位置反馈及连接。C1为0时，也需检查C0是否为6（故障）。'],
        ['蓝色常亮', '持续亮蓝色', '已暂停、自由释放或阻尼释放。',
         '读取C0区分状态；暂停后用DCT继续，释放后用ULR恢复正常控制。保护故障需先解除。'],
    ]
    return '''
  <section id="lighting-guide">
   <h2>四、灯光状态说明</h2>
   <p>指示灯根据舵机状态自动切换。灯光无需设置，通信协议未提供单独选择颜色、亮度或灯效的参数。</p>
   <table><thead><tr><th>灯光</th><th>显示方式</th><th>表示的状态</th><th>查询与操作</th></tr></thead><tbody>
'''+''.join(a.row_html(row) for row in rows)+'''   </tbody></table>
   <p>同时出现多个故障时，灯光按“过温（橙）→过流（紫）→电源异常（红闪）→设备异常（绿闪）”的顺序优先显示；故障提示优先于蓝色暂停/释放提示。完整故障组合请读取C1。</p>
   <p>堵转后就地保持和位置反馈恢复没有独立灯色，按当时的工作状态显示；查询堵转请读C6，查询反馈恢复请读C0。彩虹渐变也可能出现在堵转后的保持状态。</p>
  </section>
'''


def sync(source, schemas):
    assert set(schemas) == set(a.BASE_PROJECT_NAMES)
    source = re.sub(r'\s*<section id="(?:changes-from-previous|lighting-guide)">.*?</section>', '', source, flags=re.S)
    params = {name:{f'{p["id"]:02X}':p for p in schema['params']} for name,schema in schemas.items()}
    def constant(key):
        values = {s['constants'][key] for s in schemas.values()}
        assert len(values) == 1, (key,values)
        return next(iter(values))
    tables = a.tables_of(source)
    assert len(tables) == 14, 'Review table selectors after changing the manual structure'
    flash,ram,status = (a.keyed(tables[i]) for i in (10,12,13))
    for addr in ['36','39']: flash.pop(addr,None)
    for addr in ['8D','90','8C']: ram.pop(addr,None)
    for addr,row in flash.items():
        values = {p[addr]['default'] for p in params.values()}
        row[3] = next(iter(params.values()))[addr]['size']
        if re.fullmatch(r'\d+',str(row[4])):
            assert len(values) == 1, (addr,values)
            row[4] = next(iter(values))
    for addr,hw,noun in [('30','04','速度'),('31','05','加速度'),('32','06','减速度')]:
        ratios = {100*s['default_formulas'][addr]['numerator']/s['default_formulas'][addr]['denominator'] for s in schemas.values()}
        assert len(ratios) == 1, (addr,ratios)
        flash[addr][4] = f'硬件页{hw}返回值×{next(iter(ratios)):g}%，向下取整'
        flash[addr][8] = f'限制位置运动的最大{noun}。0表示使用本机额定{noun}；设置后从下一次运动规划生效。读取本地址可查看已保存设置。'
    flash['22'][6] = '1～4、7～10；11由行程标定进入'
    flash['33'][8] = ('使加速和减速变化更平缓，数值越大，起停通常越柔和。0自动选择本机平滑时间（参考硬件页07）；'
                       '非0下限为硬件页05、06较大值除以2000后向上取整，最大200ms。下一次运动规划生效。')
    flash['34'][8] = '限制位置动作的最短时间，0关闭此限制。带时间的指令取请求时间与本设置中的较大值；实际运动还受速度、加减速度和平滑时间限制。'
    flash['35'][6:9] = ['0～'+str(constant('CFG_HOLD_KP_MAX')), '‰/°',
        '调节到位后被推离目标时的回正力度。超出允许误差范围每1°，增加P‰的标定电压；P=80时为8%。设0关闭此比例回正作用，仍保留已有托举力及I作用。设置值每10ms最多变化1，以平缓过渡。']
    flash['37'][6:9] = ['硬件页08返回值×1.5向下取整～500', '0.01°',
        '允许的位置误差半宽，50表示目标两侧各允许0.50°。实际位置在此范围内并基本静止后判定到位；保持时只对超出范围的误差施加P/I回正。下限按本机最小可靠位移计算。']
    flash['38'][6:9] = ['0～'+str(constant('CFG_HOLD_KI_MAX')), '‰/(°·s)',
        '持续偏离目标时逐步增加回正力度。超出允许误差范围每1°并持续1秒，增加I‰的标定电压。设0停止新增保持积分，保留已有托举力。积分受输出限制约束。']
    flash['44'][6] = '10～硬件页04返回值除以2向下取整'
    flash['44'][8] = '位置运动中每20ms检查一次。指令要求继续运动而实际位移不超过此值时，可能计为堵转；连续达到45设置次数后，就地保持。'
    for rt,fl in [('85','30'),('86','31'),('87','32'),('88','33'),('89','34'),('8A','35'),('8E','37'),('8F','38')]:
        f = flash[fl]
        ram[rt][3:9] = [f[3], 'Flash '+fl, '读写', f[6], f[7],
            f[8].replace('读取本地址可查看已保存设置。','')+' 本地址仅本次上电有效；读取可查看当前运行设置。']
    ram['82'][8] = '上电初值为1，上电释放或保护动作可能改为0。恢复默认后请读取C0确认实际状态；从释放恢复可用ULR或新的有效运动指令。恢复后保持当前位置，原目标需重新发送。非法值或保护未解除时返回04。'
    status['C2'][7] = '位置模式返回最新读取的位置；读取失败时保留最近有效值。电机模式返回1500。C7返回最近记录的位置。'
    status['C3'][7] = '读取当前温度，250表示25.0℃；读取失败返回-32768。C7中的温度最多每250ms更新一次。'
    status['C4'][7] = '读取最近采样的电压，700表示7.00V；C7中的电压每10ms更新一次。'
    status['C5'][7] = '最近采样的电流大小，每10ms更新一次；需结合C7的扭矩FFFF判断采样是否有效。'
    status['C7'][7] = 'CMD=02读取；成功DATA为状态码＋C7＋16字节数据，共18字节。各字段为最近记录值，可能采自不同时间。'
    device = a.keyed(tables[7])
    device['00'][4:7] = ['1', '无', 'ADC电位器位置反馈']
    device['03'][6] = '扭矩设置上限为该值×100；例如返回20时，上限为2000（20.00 kgf·cm）。'
    device['07'][6] = '本机自动平滑时间参考值；平滑时间33/88设为0时自动使用本机参数。'
    tables[7] = a.replace_body(tables[7],device.values(),True)
    for i,data in [(10,flash),(12,ram),(13,status)]:
        for number,addr in enumerate(sorted(data,key=lambda x:int(x,16)),1): data[addr][0] = number
        tables[i] = a.replace_body(tables[i],data.values(),True)
    commands = a.rows_of(tables[8])
    for row in commands:
        if row[0] == '多圈运动':
            row[2] = '本表所列ADC型号不支持此格式，不执行也不应答；位置运动请使用不带N字段的指令。'
        elif row[0] == '位置/速度运动':
            row[2] = '位置模式目标0500～2500，T为0000～9999ms，0表示按最快允许轨迹运动。实际时间受速度、加减速度、平滑时间和最短运动时间限制。电机模式前4位为驱动大小及方向，1500为零驱动；7/8的T为圈数，9/10为秒，T=0无限。'
        elif row[0] == '设置工作模式':
            row[2] = '切换并保存工作模式：1～4为位置模式，7/8为定圈模式，9/10为定时模式；5/6不可设置。11由行程标定自动进入。各模式含义见“舵机工作模式说明”。'
        elif row[0] == '设置扭矩上限':
            row[2] = '设置输出扭矩上限，固定五位数，单位0.01 kgf·cm；范围0至本机硬件扭矩×100。立即生效并保存；示例设置10.00 kgf·cm。'
        elif row[0] == '查询当前位置':
            row[2] = '位置模式返回最新读取的位置；读取失败保留最近有效值。500～2500是位置刻度，1500表示中位。电机模式返回1500。'
    tables[8] = a.replace_body(tables[8],commands)
    modes = a.rows_of(tables[11])
    for row in modes:
        if row[0] in ['5','6']: row[2] = '本表所列ADC型号不可设置。'
    tables[11] = a.replace_body(tables[11],modes)
    it = iter(tables)
    result = re.sub(r'<table\b[^>]*>.*?</table>',lambda _:next(it),source,flags=re.S)
    return result.replace(' </body>',lighting_section()+' </body>')


def main():
    a.REVIEW.mkdir(parents=True,exist_ok=True)
    schemas = {name:a.probe(a.ROOT/name) for name in a.BASE_PROJECT_NAMES}
    result = sync(a.DOC.read_text(encoding='utf-8'),schemas)
    for name,schema in schemas.items():
        issues,_ = a.audit(result,schema)
        assert not issues, (name,issues)
    a.DOC.write_text(result,encoding='utf-8',newline='\n')
    print('PASS: user reference and LED guide verified against ADC 15kg, 20kg and 30kg')


if __name__ == '__main__': main()
