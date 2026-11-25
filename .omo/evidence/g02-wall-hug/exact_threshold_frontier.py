"""Enumerate every distinct monotone response/variance/chain split of 16 rows."""
import itertools
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent/"offline-gate-review"
rows=json.loads((ROOT/"all-accepted-edges.json").read_text())
coarse=np.array([r["coarse"] for r in rows])
fine=np.array([r["fine"] for r in rows])
variance=np.array([max(r["coarse_covariance"][0],r["coarse_covariance"][4]) for r in rows])
chain=np.array([r["chain_size"] for r in rows])
correct=np.array([r["correct"] for r in rows])
# Tighten only from the original .35/.45/9m²/10 values. Strict/non-strict
# conditions exactly match Karto's > coarse, >= fine, < variance, >= chain.
coarse_thresholds=[.35,*sorted(set(coarse))]
fine_thresholds=[.45,*sorted(set(fine))]
variance_thresholds=[9.,*sorted(set(variance))]
chain_thresholds=[10,*sorted(set(chain)),int(chain.max())+1]
best=-1; winners=[]
for c,f,v,n in itertools.product(
        coarse_thresholds,fine_thresholds,variance_thresholds,chain_thresholds):
    keep=(coarse>c)&(fine>=f)&(variance<v)&(chain>=n)
    if np.any(keep&~correct):
        continue
    retained=int(np.sum(keep&correct))
    if retained<best:
        continue
    row=dict(coarse_gt=float(c),fine_gte=float(f),variance_lt=float(v),chain_gte=int(n),
             wrong_rejected=6,correct_retained=retained,correct_lost=10-retained)
    if retained>best:
        best=retained;winners=[]
    winners.append(row)
summary=dict(best_correct_retained=best,correct_lost=10-best,
             threshold_sets=len(winners),first_sets=winners[:12],
             evaluated_splits=len(coarse_thresholds)*len(fine_thresholds)*
                 len(variance_thresholds)*len(chain_thresholds))
(ROOT/"exact-config-frontier.json").write_text(json.dumps(summary,indent=2))
print("EXACT_CONFIG_FRONTIER "+json.dumps(summary),flush=True)
