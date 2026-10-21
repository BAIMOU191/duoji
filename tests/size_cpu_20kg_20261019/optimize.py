from pathlib import Path
import importlib.util,json,hashlib,shutil,sys,re
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PROJECT=ROOT/'CH32V00X_APP_ADC_20kg'
OUT=HERE/'output'
FOLDERS=('Application','Common','Drivers','System','User','SRC')

POLY='''/* 同向接续的三次速度段。调用方vmax<=6553600(Q16)，an限于27位；
 * 先检查端点范围后，系数及Bernstein控制点均可用int32精确计算。 */
static uint8_t Traj_PolyMake(TrajSeg_t *g, int32_t v0, int32_t an, int32_t v1,
                             int32_t n, int32_t vmax)
{
    int32_t dv, b, c, d, av[3], bv[4], j0, j1;
    int64_t den;
    uint8_t i;

    /* 量化后的末速度与v1相差至多6；提前剔除必不满足原包络检查的端点。 */
    if (n < 8 || v0 < -4 || v0 > vmax + 4 || v1 < -10 || v1 > vmax + 10
        || an < -0x07FFFFFF || an > 0x07FFFFFF) return 0U;
    dv = v1 - v0;
    b = an / 2; c = (3 * dv - 2 * an) / 3; d = (-2 * dv + an) / 4;
    if (c < -0x07FFFFFF || c > 0x07FFFFFF || d < -0x07FFFFFF || d > 0x07FFFFFF) return 0U;
    bv[0] = v0; bv[1] = v0 + 2 * b / 3;
    bv[2] = v0 + 4 * b / 3 + c; bv[3] = v0 + 2 * b + 3 * c + 4 * d;
    av[0] = 2 * b; av[1] = 2 * b + 3 * c; av[2] = 2 * b + 6 * c + 12 * d;
    for (i = 0U; i < 4U; i++) if (bv[i] < -4 || bv[i] > vmax + 4) return 0U;
    for (i = 0U; i < 3U; i++)
        if ((int64_t)av[i] * 1000000 > (int64_t)s_tj.amax_cdpss * n * 65536
            || (int64_t)av[i] * 1000000 < -(int64_t)s_tj.dmax_cdpss * n * 65536) return 0U;
    /* jerk线性：只比较两端最大值。1e9/65536约成1953125/128，商与原式完全相同。 */
    j0 = Fx_Abs32(6 * c); j1 = Fx_Abs32(6 * c + 24 * d);
    if (j1 > j0) j0 = j1;
    den = (int64_t)n * n * 128;
    if (Traj_DivSat((int64_t)j0 * 1953125 + den - 1, den) > s_tj.jmax_cdpss3) return 0U;
    g->v0_qt = v0; g->r_q15 = b; g->peak_q15 = c; g->inv_r_q15 = d;
    g->posk = n;
    return 1U;
}

/* 私有求值只接收0<=tick<=段长，tick*floor(2^30/段长)<=2^30，不需要64位乘法。 */
static int32_t Traj_PolyU(const TrajSeg_t *g, int32_t tick)
{
    return (int32_t)(((uint32_t)tick * (uint32_t)g->inv_q30) >> 14);
}

static int32_t Traj_PolyPos(const TrajSeg_t *g, int32_t u)
{
    int32_t p = g->peak_q15 + (int32_t)(((int64_t)g->inv_r_q15 * u) >> 16);
    p = g->r_q15 + (int32_t)(((int64_t)p * u) >> 16);
    p = g->v0_qt + (int32_t)(((int64_t)p * u) >> 16);
    p = (int32_t)(((int64_t)p * u) >> 16);
    return g->p0 + (int32_t)(((int64_t)p * g->posk) >> 16);
}

static void Traj_PolyEval(const TrajSeg_t *g, int32_t tick,
                          int32_t *pos, int32_t *vel, int32_t *acc)
{
    int32_t u = Traj_PolyU(g, tick);
    int32_t b = g->r_q15, c = g->peak_q15, d = g->inv_r_q15, v;
    int64_t a;

    v = 3 * c + (int32_t)(((int64_t)(4 * d) * u) >> 16);
    v = 2 * b + (int32_t)(((int64_t)v * u) >> 16);
    v = g->v0_qt + (int32_t)(((int64_t)v * u) >> 16);
    a = 6LL * c + (((int64_t)(12 * d) * u) >> 16);
    a = 2LL * b + (a * u >> 16);
    *pos = Traj_PolyPos(g, u);
    *vel = v;
    *acc = (int32_t)((a * g->acck) >> 30);
}

'''

def snapshot():
    if (HERE/'source_hashes_before.json').exists():return
    manifest={f.relative_to(ROOT).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for p in ROOT.glob('CH32V00X_APP*') if p.is_dir() for d in FOLDERS for f in (p/d).rglob('*') if f.is_file()}
    (HERE/'source_hashes_before.json').write_text(json.dumps(manifest,indent=2))
    for name in ('CH32V00X_APP_ADC_20kg','CH32V00X_APP_ADC_30kg'):
        src=ROOT/name;dest=HERE/'before'/name
        for folder in (*FOLDERS,'obj'):
            if (src/folder).exists():shutil.copytree(src/folder,dest/folder)
        for f in ('.cproject','CH32V00X_APP.wvproj'):
            shutil.copy2(src/f,dest/f)

def candidate():
    p=HERE/'candidate'
    if not p.exists():
        for folder in FOLDERS:shutil.copytree(HERE/'before'/PROJECT.name/folder,p/folder)
    text=(HERE/'before'/PROJECT.name/'Common/C_Traj_Planner.c').read_text(encoding='utf-8')
    a=text.index('/* 三次速度段：');b=text.index('static uint8_t Traj_TimedJoin',a)
    text=text[:a]+POLY+text[b:]
    text=text.replace('int32_t n = (T * percent + 99) / 100, vc, pa, pc, end, v, a;', 'int32_t n = (T * percent + 99) / 100, vc, pa, pc, end;')
    text=text.replace('int64_t an = (int64_t)a0 * n * 65536 / 1000000;', 'int32_t an = Traj_DivSat((int64_t)a0 * n * 1024, 15625);')
    text=text.replace(' - an * n / 12;', ' - (int64_t)an * n / 12;')
    text=text.replace('if (n < 8 || T <= 2 * n || numerator <= 0) continue;', 'if (n < 8 || T <= 2 * n || numerator <= 0 || Fx_Abs32(an) > 0x07FFFFFF) continue;')
    text=text.replace('Traj_PolyMake(&g[0], v0, a0, vc, n, vmax)', 'Traj_PolyMake(&g[0], v0, an, vc, n, vmax)')
    text=text.replace('''        g[0].p0 = 0;
        Traj_PolyEval(&g[0], n, &pa, &v, &a);''','''        /* 两段同长，倒数各算一次；端点只求位置，不做无用的速度/加速度计算。 */
        g[0].inv_q30 = g[1].inv_q30 = Traj_DivSat(1LL << 30, n);
        g[0].acck = g[1].acck = Traj_DivSat(1000000LL << 14, n);
        g[0].p0 = 0;
        pa = Traj_PolyPos(&g[0], Traj_PolyU(&g[0], n));''')
    text=text.replace('Traj_PolyEval(&g[1], n, &end, &v, &a);','end = Traj_PolyPos(&g[1], Traj_PolyU(&g[1], n));')
    (p/'Common/C_Traj_Planner.c').write_text(text,encoding='utf-8')
    return p

if __name__=='__main__':
    HERE.mkdir(exist_ok=True);OUT.mkdir(exist_ok=True);snapshot();print(candidate())
