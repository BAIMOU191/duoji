"""Reproduce the isolated experiment, never writing production firmware."""
import json
import sys
import run_optimization as r
import verify_candidate as v
import extra_boundaries

r.prepare()
selected=['baseline','inertia1','optimized','handoff']
r.matrix(None if '--full' in sys.argv else selected)
r.extra(['baseline','inertia1','optimized','opt_j0','opt_j2','opt_j4','opt_hold_i10','opt_hold_i30','opt_hold_i60','handoff'] if '--full' in sys.argv else selected)
r.traces(selected)
extra_boundaries.run()
v.boundary(['handoff','limits'])
v.target(['baseline','handoff'])
r.verify()
matrix=json.loads((r.OUT/'matrix.json').read_text())
(r.OUT/'final_matrix.json').write_text(json.dumps({p:matrix[p] for p in selected},indent=2),encoding='utf-8')
import make_report
make_report.plots();make_report.report(make_report.export())
