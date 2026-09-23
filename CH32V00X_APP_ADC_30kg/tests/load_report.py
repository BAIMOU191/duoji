"""生成改动前后的带载仿真对比页(自包含 HTML + 内联 SVG，无第三方依赖)。

用法：python tests/load_report.py <改动前源码目录>
      改动后 = 本工程源码；对象模型取本工程 tests/sim。输出 tests/results/load_matrix/load_compare.html
"""
import csv, html, io, json, pathlib, subprocess, sys
import load_matrix as LM

HERE = pathlib.Path(__file__).resolve().parent
OUT = LM.OUT / 'load_compare.html'

# (标题, 场景, 负载名, 起止ms, 纵轴量)
PANELS = [
    ('小角度 5° 往返 · 空载', 'small', '空载', (8200, 12300), 'pos'),
    ('小角度 5° 往返 · 带载', 'small', '摆臂800+惯量x3', (8200, 12300), 'pos'),
    ('大角度 270° 到位段(放大) · 空载', 'large', '空载', (13500, 14800), 'pos'),
    ('大角度 270° 到位段(放大) · 带载', 'large', '摆臂800+惯量x3', (13500, 14800), 'pos'),
    ('来回转 ±40°/500ms · 空载', 'recip', '空载', (3300, 8000), 'pos'),
    ('来回转 ±40°/500ms · 带载', 'recip', '摆臂800+惯量x3', (3300, 8000), 'pos'),
    ('带载保持时的PWM(小角度 5°)', 'small', '摆臂800+惯量x3', (8200, 12300), 'pwm'),
    ('带载保持时的PWM · Stribeck摩擦', 'small', '摆臂600+Stribeck', (8200, 12300), 'pwm'),
]


def trace(exe, scen, load):
    p = dict(LM.LOADS[load], seed=1, trace=1)
    r = subprocess.run([str(exe), scen] + [f'{k}={v}' for k, v in p.items()],
                       capture_output=True, text=True, check=True)
    return list(csv.DictReader(io.StringIO(r.stdout)))


def series(rows, t0, t1, key, step=4):
    out = []
    for r in rows[t0:t1:step]:
        if key == 'pos':
            out.append(round(float(r['pos']) / 100, 2))
        else:
            out.append(int(r['pwm']))
    return out


def main():
    old_root = pathlib.Path(sys.argv[1])
    exe_old = LM.build(old_root, 'report_old')
    exe_new = LM.build(HERE.parent, 'report_new')
    panels = []
    cache = {}
    for title, scen, load, (t0, t1), key in PANELS:
        for tag, exe in (('old', exe_old), ('new', exe_new)):
            if (tag, scen, load) not in cache:
                cache[(tag, scen, load)] = trace(exe, scen, load)
        ro, rn = cache[('old', scen, load)], cache[('new', scen, load)]
        # 目标线：用指令换算的目标(两版相同)，取新版 ref 在轨迹走完后的值
        cmd = [round(LM_target(r) / 100, 2) for r in rn[t0:t1:4]]
        panels.append({'title': title, 'key': key, 't0': t0, 'step': 4,
                       'old': series(ro, t0, t1, key), 'new': series(rn, t0, t1, key),
                       'target': cmd if key == 'pos' else None})
    metrics = {}
    # 全矩阵结果：改动前取 --root 跑出的 rob_old_nom.json(或 old.json)，改动后取 final.json
    for tag, names in (('old', ('rob_old_nom.json', 'old.json')), ('final', ('final.json',))):
        for nm in names:
            p = LM.OUT / nm
            if p.exists():
                metrics[tag] = json.loads(p.read_text(encoding='utf-8'))['cases']
                break
    OUT.write_text(render(panels, metrics), encoding='utf-8')
    print(OUT)


def LM_target(r):
    # 目标角度：servo_sim 的 target_of 与 CFG_DIR_INVERT=1 一致(脉宽增大角度减小)
    pwm = int(r['cmd']) if r['cmd'] != '0' else 1500
    return 27000 - (pwm - 500) * 27000 / 2000.0


def group_rows(metrics):
    """按负载组汇总：超调均值、到位时间均值、保持峰峰均值、运动PWM活动均值"""
    if 'old' not in metrics or 'final' not in metrics:
        return []
    groups = {}
    for case in metrics['old']:
        load = case.split('|')[0]
        groups.setdefault(load, []).append(case)
    rows = []
    for load, cases in groups.items():
        def avg(tag, m):
            return sum(metrics[tag][c][m] for c in cases) / len(cases)
        rows.append((load, [(avg('old', m), avg('final', m)) for m in
                            ('over_max', 'settle_max', 'hold_pp', 'move_dpwm')]))
    return rows


def render(panels, metrics):
    data = json.dumps(panels, ensure_ascii=False)
    trs = []
    for load, vals in group_rows(metrics):
        tds = ''.join(f'<td class="num">{o:.0f}</td><td class="num new">{n:.0f}</td>'
                      if i < 3 else f'<td class="num">{o:.1f}</td><td class="num new">{n:.1f}</td>'
                      for i, (o, n) in enumerate(vals))
        trs.append(f'<tr><th scope="row">{html.escape(load)}</th>{tds}</tr>')
    table = ''.join(trs)
    return TEMPLATE.replace('__DATA__', data).replace('__TABLE__', table)


TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>30kg带载仿真对比</title>
<style>
:root{color-scheme:light;--surface:#fcfcfb;--text:#0b0b0b;--text2:#52514e;--muted:#8a8984;--grid:#e8e7e3;
--s-new:#2a78d6;--s-old:#eb6834;}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])){color-scheme:dark;--surface:#1a1a19;--text:#fff;--text2:#c3c2b7;--muted:#8f8e86;--grid:#2e2e2c;--s-new:#3987e5;--s-old:#d95926;}}
:root[data-theme="dark"]{color-scheme:dark;--surface:#1a1a19;--text:#fff;--text2:#c3c2b7;--muted:#8f8e86;--grid:#2e2e2c;--s-new:#3987e5;--s-old:#d95926;}
body{margin:0;background:var(--surface);color:var(--text);font:14px/1.5 system-ui,"Microsoft YaHei",sans-serif}
main{max-width:1180px;margin:0 auto;padding:20px 16px 40px}
h1{font-size:20px;margin:0 0 4px}p.sub{color:var(--text2);margin:0 0 16px}
.legend{display:flex;gap:18px;align-items:center;color:var(--text2);margin:0 0 12px;flex-wrap:wrap}
.sw{display:inline-block;width:18px;height:2px;vertical-align:middle;margin-right:6px}
.sw.dash{background:none;border-top:2px dashed var(--muted);height:0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:16px}
figure{margin:0;border:1px solid var(--grid);border-radius:8px;padding:10px 10px 6px;position:relative}
figcaption{font-weight:600;margin:0 0 4px}
svg{width:100%;height:auto;display:block;overflow:visible}
.axis text{fill:var(--muted);font-size:11px}.axis line{stroke:var(--grid)}
.tip{position:absolute;pointer-events:none;background:var(--surface);border:1px solid var(--grid);border-radius:6px;
padding:6px 8px;font-size:12px;color:var(--text);display:none;white-space:nowrap;box-shadow:0 2px 8px rgba(0,0,0,.12)}
.tip i{display:inline-block;width:10px;height:2px;margin-right:5px;vertical-align:middle}
table{border-collapse:collapse;width:100%;margin-top:10px;font-size:13px}
th,td{padding:6px 8px;border-bottom:1px solid var(--grid);text-align:left}
td.num{text-align:right;font-variant-numeric:tabular-nums;color:var(--text2)}td.new{color:var(--text);font-weight:600}
thead th{color:var(--text2);font-weight:600}
.scroll{overflow-x:auto}
.note{color:var(--text2);font-size:13px;margin-top:14px}
</style></head><body><main>
<h1>30kg舵机 带载仿真：改动前 vs 改动后</h1>
<p class="sub">同一被控对象(11:24标定 + 带载模型)，只换控制固件。“带载”= 摆臂重力800 PWM当量(约1/3额定扭矩) + 外加惯量3倍空载惯量。</p>
<div class="legend"><span><span class="sw" style="background:var(--s-old)"></span>改动前(当前固件)</span>
<span><span class="sw" style="background:var(--s-new)"></span>改动后</span>
<span><span class="sw dash"></span>目标角度</span></div>
<div class="grid" id="charts"></div>
<h2 style="font-size:16px;margin:28px 0 0">全矩阵汇总(每组 6 个场景 × 2 个随机种子取最坏)</h2>
<div class="scroll"><table><thead><tr><th>负载工况</th><th colspan="2">超调 厘度(前/后)</th><th colspan="2">到位 ms(前/后)</th>
<th colspan="2">保持峰峰 厘度(前/后)</th><th colspan="2">运动中|ΔPWM|(前/后)</th></tr></thead><tbody>__TABLE__</tbody></table></div>
<p class="note">仿真是简化模型(一阶电机+Stribeck摩擦+重力+惯量，可选齿轮柔性)，用于同条件下比较方案，不等于实机保证。运动中|ΔPWM|是每拍PWM变化量的平均，反映电机“沙沙声”(见表最后两列)。</p>
</main>
<script>
const P=__DATA__;
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function draw(){
 const root=document.getElementById('charts');root.innerHTML='';
 P.forEach(p=>{
  const W=520,H=230,L=48,R=10,T=10,B=26;
  const fig=document.createElement('figure');fig.innerHTML='<figcaption>'+p.title+'</figcaption>';
  const vals=p.old.concat(p.new).concat((p.target||[]).filter(v=>v!==null));
  let lo=Math.min(...vals),hi=Math.max(...vals);const pad=(hi-lo)*0.08||1;lo-=pad;hi+=pad;
  const n=p.old.length,x=i=>L+(W-L-R)*i/(n-1),y=v=>T+(H-T-B)*(1-(v-lo)/(hi-lo));
  const NS='http://www.w3.org/2000/svg',svg=document.createElementNS(NS,'svg');svg.setAttribute('viewBox',`0 0 ${W} ${H}`);
  svg.setAttribute('role','img');svg.setAttribute('aria-label',p.title);
  const g=document.createElementNS(NS,'g');g.setAttribute('class','axis');
  for(let k=0;k<=4;k++){const v=lo+(hi-lo)*k/4,yy=y(v);
   g.innerHTML+=`<line x1="${L}" x2="${W-R}" y1="${yy}" y2="${yy}"/><text x="${L-6}" y="${yy+4}" text-anchor="end">${v.toFixed(p.key==='pos'?1:0)}</text>`;}
  const dur=(n-1)*p.step;for(let k=0;k<=4;k++){const xx=L+(W-L-R)*k/4;g.innerHTML+=`<text x="${xx}" y="${H-6}" text-anchor="middle">${(dur*k/4/1000).toFixed(1)}s</text>`;}
  g.innerHTML+=`<text x="${L}" y="${T-0}" dy="-1" fill="var(--muted)"> </text>`;
  svg.appendChild(g);
  const path=(arr,color,w,dash)=>{let d='';arr.forEach((v,i)=>{if(v===null)return;d+=(d&&arr[i-1]!==null?'L':'M')+x(i).toFixed(1)+','+y(v).toFixed(1);});
   const e=document.createElementNS(NS,'path');e.setAttribute('d',d);e.setAttribute('fill','none');e.setAttribute('stroke',color);
   e.setAttribute('stroke-width',w);e.setAttribute('stroke-linejoin','round');if(dash)e.setAttribute('stroke-dasharray',dash);svg.appendChild(e);};
  if(p.target)path(p.target,css('--muted'),1.5,'4 3');
  path(p.old,css('--s-old'),2);path(p.new,css('--s-new'),2);
  const unit=p.key==='pos'?'°':' PWM';
  svg.insertAdjacentHTML('beforeend',`<text x="${L+4}" y="${T+12}" fill="${css('--muted')}" font-size="11">${p.key==='pos'?'角度(°)':'PWM'}</text>`);
  const cross=document.createElementNS(NS,'line');cross.setAttribute('stroke',css('--muted'));cross.setAttribute('y1',T);cross.setAttribute('y2',H-B);cross.style.display='none';svg.appendChild(cross);
  const hit=document.createElementNS(NS,'rect');hit.setAttribute('x',L);hit.setAttribute('y',T);hit.setAttribute('width',W-L-R);hit.setAttribute('height',H-T-B);hit.setAttribute('fill','transparent');svg.appendChild(hit);
  const tip=document.createElement('div');tip.className='tip';fig.appendChild(svg);fig.appendChild(tip);
  hit.addEventListener('mousemove',ev=>{const r=svg.getBoundingClientRect(),sx=(ev.clientX-r.left)*W/r.width;
   const i=Math.max(0,Math.min(n-1,Math.round((sx-L)/(W-L-R)*(n-1))));cross.setAttribute('x1',x(i));cross.setAttribute('x2',x(i));cross.style.display='';
   tip.innerHTML=`<b>${(i*p.step/1000).toFixed(3)} s</b><br><i style="background:${css('--s-old')}"></i>改动前 ${p.old[i]}${unit}<br><i style="background:${css('--s-new')}"></i>改动后 ${p.new[i]}${unit}`+(p.target&&p.target[i]!==null?`<br>目标 ${p.target[i]}°`:'');
   tip.style.display='block';const fx=ev.clientX-fig.getBoundingClientRect().left;tip.style.left=Math.min(fx+12,fig.clientWidth-150)+'px';tip.style.top='34px';});
  hit.addEventListener('mouseleave',()=>{tip.style.display='none';cross.style.display='none';});
  root.appendChild(fig);});
}
draw();matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);
</script></body></html>
'''

if __name__ == '__main__':
    main()
