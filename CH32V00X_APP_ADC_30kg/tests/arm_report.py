"""生成肩关节(机械臂第二关节)仿真对比页：上一版 vs 本轮，自包含 HTML。

用法：python tests/arm_report.py <上一版源码目录>
      本轮 = 本工程源码；对象模型 tests/sim/arm_sim.c。输出 tests/results/arm_matrix/arm_compare.html
      汇总表读 results/arm_matrix/dense_<old|prev|final>.json(按结构摆动频率分组)，没有就不出表。
"""
import collections, csv, html, io, json, pathlib, statistics, subprocess, sys
import arm_matrix as AM

HERE = pathlib.Path(__file__).resolve().parent
OUT = AM.OUT / 'arm_compare.html'
SOFT = {'inertia': 3, 'farm': 4, 'zarm': 0.08, 'gap': 40, 'grav': 900}
MID = {'inertia': 2, 'farm': 6, 'zarm': 0.05, 'gap': 30, 'grav': 1000}
# (标题, 场景, 手臂参数, 起止ms, 纵轴)
PANELS = [
    ('刚性连接 · 轻推(400)后松手', 'push', {'farm': 0, 'inertia': 3, 'grav': 0, 'pushf': 400}, (3500, 9500), 'dev'),
    ('刚性连接 · 手推(800)后松手', 'push', {'farm': 0, 'inertia': 3, 'grav': 0, 'pushf': 800}, (3500, 9500), 'dev'),
    ('软结构(约4Hz)带重力 · 手推(800)后松手', 'push', dict(SOFT, pushf=800), (3500, 9500), 'dev'),
    ('软结构(约4Hz) · 主机插补结束后保持', 'stream', SOFT, (10000, 13000), 'dev'),
    ('中等结构(约6Hz) · 前倾20°保持', 'hold', MID, (7100, 10000), 'dev'),
    ('软结构(约4Hz) · 竖直附近保持', 'hold', SOFT, (2600, 5500), 'dev'),
]
STEP = 4


def trace(exe, scen, params):
    r = subprocess.run([str(exe), scen] + [f'{k}={v}' for k, v in params.items()] + ['trace=1'],
                       capture_output=True, text=True, check=True)
    return list(csv.DictReader(io.StringIO(r.stdout)))


def target(r):
    # 指令脉宽 -> 行程坐标(CFG_DIR_INVERT=1：脉宽增大角度减小)，度
    return round((27000 - (int(r['cmd']) - 500) * 27000 / 2000.0) / 100, 2)


def main():
    exe_old = AM.build(pathlib.Path(sys.argv[1]), 'report_prev')
    exe_new = AM.build(HERE.parent, 'report_final')
    panels = []
    for title, scen, par, (t0, t1), key in PANELS:
        ro, rn = trace(exe_old, scen, par), trace(exe_new, scen, par)
        if key == 'dev':      # 手臂相对目标的偏差，度；手推场景翻成"推的方向为正"
            sg = -1 if scen == 'push' else 1
            col = lambda r, sg=sg: round(sg * (float(r['arm']) / 100 - target(r)), 3)
        else:
            col = lambda r: int(r['pwm'])
        panels.append({'title': title, 'key': 'pos' if key == 'dev' else key, 'step': STEP,
                       'old': [col(r) for r in ro[t0:t1:STEP]], 'new': [col(r) for r in rn[t0:t1:STEP]],
                       'target': [0.0] * len(ro[t0:t1:STEP]) if key == 'dev' else None})
    rows = []
    dens = {l: AM.OUT / f'dense_{l}.json' for l in ('prev', 'H_cur', 'final3')}
    if all(p.exists() for p in dens.values()):
        R = {l: json.loads(p.read_text(encoding='utf-8')) for l, p in dens.items()}
        bad = lambda v: v['end_pp'] > 50 or (v['growth'] > 1.2 and v['end_pp'] > 20)
        freqs = sorted({int(k.split('|')[1].split('=')[1]) for k in R['final3']})
        for f in freqs + ['全部']:
            ks = [k for k in R['final3'] if f == '全部' or k.split('|')[1] == f'farm={f}']
            cells = ''.join(f'<td class="num{" new" if l == "final3" else ""}">{sum(bad(R[l][k]) for k in ks)}/{len(ks)}</td>' for l in ('prev', 'H_cur', 'final3'))
            cells += ''.join(f'<td class="num{" new" if l == "final3" else ""}">{statistics.mean(R[l][k]["end_pp"] for k in ks):.0f}</td>' for l in ('prev', 'H_cur', 'final3'))
            rows.append(f'<tr><th scope="row">{f if f == "全部" else str(f) + " Hz"}</th>{cells}</tr>')
    page = TEMPLATE.replace('__DATA__', json.dumps(panels, ensure_ascii=False)).replace('__TABLE__', ''.join(rows))
    page = page.replace('__SUB__', SUB).replace('__H2__', H2).replace('__THEAD__', THEAD).replace('__NOTE__', NOTE)
    OUT.write_text(page, encoding='utf-8')
    print(OUT)


SUB = ('纵轴是<b>手臂</b>相对目标的偏差(度，用户看到的)，不是舵机输出轴；横轴从保持段开始。模型：舵机输出轴经"结构弹簧+间隙"带动整条手臂，重力为倒立摆(竖直为0，重心前倾30°)。'
       '软结构=手臂惯量3倍舵机惯量、摆动频率约4Hz、间隙0.4°、重力矩900；中等=2倍、约6Hz、0.3°、1000。')
H2 = '全网格汇总(960 个手臂工况：惯量 × 结构频率 × 阻尼 × 间隙 × 重力，保持/慢动/插补三场景)'
THEAD = ('<tr><th>结构摆动频率</th><th colspan="3">"抖"的工况数(很抖的那版/你现在刷的/本轮)</th>'
         '<th colspan="3">保持末段手臂峰峰 厘度(很抖的那版/你现在刷的/本轮)</th></tr>')
NOTE = ('"抖"=保持段最后三分之一手臂峰峰超过50厘度(0.5°)，或摆动还在变大。手推图的纵轴是手臂偏离推之前位置的角度(推的方向为正)，'
        '4.0~6.8s 推住，7.0s 松手；松手后冲到负值就是"反冲"。简化模型，只用于同条件下比较方案；保持刚度越高越容易把软手臂推起来摆，'
        '本轮取 TUNE_HOLD_STIFF=2 的折中。')

TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>肩关节手臂仿真</title>
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
<h1>30kg舵机 · 到位保持刚度：你现在刷的版本 vs 本轮</h1>
<p class="sub">__SUB__</p>
<div class="legend"><span><span class="sw" style="background:var(--s-old)"></span>上一版(你现在刷的)</span>
<span><span class="sw" style="background:var(--s-new)"></span>本轮</span>
<span><span class="sw dash"></span>目标(偏差0)</span></div>
<div class="grid" id="charts"></div>
<h2 style="font-size:16px;margin:28px 0 0">__H2__</h2>
<div class="scroll"><table><thead>__THEAD__</thead><tbody>__TABLE__</tbody></table></div>
<p class="note">__NOTE__</p>
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
  svg.insertAdjacentHTML('beforeend',`<text x="${L+4}" y="${T+12}" fill="${css('--muted')}" font-size="11">${p.key==='pos'?'手臂偏差(°)':'PWM'}</text>`);
  const cross=document.createElementNS(NS,'line');cross.setAttribute('stroke',css('--muted'));cross.setAttribute('y1',T);cross.setAttribute('y2',H-B);cross.style.display='none';svg.appendChild(cross);
  const hit=document.createElementNS(NS,'rect');hit.setAttribute('x',L);hit.setAttribute('y',T);hit.setAttribute('width',W-L-R);hit.setAttribute('height',H-T-B);hit.setAttribute('fill','transparent');svg.appendChild(hit);
  const tip=document.createElement('div');tip.className='tip';fig.appendChild(svg);fig.appendChild(tip);
  hit.addEventListener('mousemove',ev=>{const r=svg.getBoundingClientRect(),sx=(ev.clientX-r.left)*W/r.width;
   const i=Math.max(0,Math.min(n-1,Math.round((sx-L)/(W-L-R)*(n-1))));cross.setAttribute('x1',x(i));cross.setAttribute('x2',x(i));cross.style.display='';
   tip.innerHTML=`<b>${(i*p.step/1000).toFixed(3)} s</b><br><i style="background:${css('--s-old')}"></i>上一版 ${p.old[i]}${unit}<br><i style="background:${css('--s-new')}"></i>本轮 ${p.new[i]}${unit}`+'';
   tip.style.display='block';const fx=ev.clientX-fig.getBoundingClientRect().left;tip.style.left=Math.min(fx+12,fig.clientWidth-150)+'px';tip.style.top='34px';});
  hit.addEventListener('mouseleave',()=>{tip.style.display='none';cross.style.display='none';});
  root.appendChild(fig);});
}
draw();matchMedia('(prefers-color-scheme: dark)').addEventListener('change',draw);
</script></body></html>
'''

if __name__ == '__main__':
    main()
