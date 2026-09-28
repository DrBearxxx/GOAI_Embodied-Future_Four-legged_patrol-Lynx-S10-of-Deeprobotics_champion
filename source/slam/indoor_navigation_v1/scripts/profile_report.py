import pstats,json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
profiles=sorted((root/'results/latency_v2').glob('worker-*.prof'))
if profiles:
    f=profiles[-1];print(f.name);pstats.Stats(str(f)).strip_dirs().sort_stats('cumulative').print_stats(35)
logs=sorted((root/'results/latency_v2').glob('localize-*.jsonl'))
if logs:
    f=logs[-1];a=[json.loads(x) for x in f.read_text().splitlines()];a=[x['pipeline'] for x in a if 'pipeline' in x]
    print(f.name)
    for k in a[0] if a else []:
        v=[x[k] for x in a if x[k] is not None]
        print(k,dict(median=float(np.median(v)),p95=float(np.quantile(v,.95)),max=float(np.max(v))))
