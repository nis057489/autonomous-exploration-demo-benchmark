"""How much must be re-sent per map update, under different unit schemes?

Replays New College in real observation order (one pose-graph scan at a time)
and, after each update, asks the question the 2D scheduler asks: which (unit,
band) fingerprints changed, and how many bytes do those dirty bands cost?

Schemes differ only in what a "unit" is:
  global  three channels spanning the whole map      (what 3D mode does today)
  slab-N  split by x into N coordinate ranges, 3 channels each
  cube-B  split into B-metre bricks, 3 channels each
"""
import numpy as np, sys, json

RES = 0.2
UPDATES = 40

def haar_bands(seq, L=None):
    """Forward Haar, returned as a list of per-band coefficient arrays,
    coarsest first -- matching haarForward's [s_L | d_L | ... | d_1] layout."""
    n = len(seq)
    if n < 2: return [np.asarray(seq, dtype=np.int64)]
    if L is None:                       # haarAutoLevels: smooth band ~2-4 values
        L, m = 0, n
        while m > 4: m = (m+1)//2; L += 1
        L = max(L, 1)
    c = np.asarray(seq, dtype=np.int64).copy()
    sl = [n]
    for _ in range(L): sl.append((sl[-1]+1)//2)
    for lvl in range(L):
        m = sl[lvl]; npair = m//2; ns = (m+1)//2
        a = c[0:2*npair:2].copy(); b = c[1:2*npair:2].copy()
        d = b - a; s = a + np.floor_divide(d, 2)
        sm = np.empty(ns, dtype=np.int64); sm[:npair] = s
        if m % 2: sm[ns-1] = c[m-1]
        c[:ns] = sm; c[ns:m] = d
    bounds = [0, sl[L]] + [sl[L-j] for j in range(1, L+1)]
    return [c[bounds[i]:bounds[i+1]] for i in range(len(bounds)-1)]

def varint_bytes(a):
    """Exact zigzag-varint length, vectorised."""
    a = np.asarray(a, dtype=np.int64)
    if a.size == 0: return 0
    zz = np.where(a >= 0, a.astype(np.uint64)*2, ((-(a+1)).astype(np.uint64))*2+1)
    out = np.ones(zz.shape, dtype=np.int64)
    cur = zz.copy()
    for _ in range(9):
        cur = cur >> np.uint64(7)
        out += (cur > 0)
    return int(out.sum())

def units_for(scheme, X, Y, Z):
    """Yields (unit_key, x, y, z) triples."""
    if scheme == "global":
        yield ("g", X, Y, Z); return
    kind, param = scheme.split("-")
    if kind == "slab":
        nslab = int(param)
        if len(X) == 0: return
        edges = np.linspace(X.min(), X.max()+1, nslab+1)
        idx = np.clip(np.searchsorted(edges, X, side="right")-1, 0, nslab-1)
        for s in range(nslab):
            m = idx == s
            if m.any(): yield (f"s{s}", X[m], Y[m], Z[m])
    else:  # cube-B metres
        cells = max(1, int(float(param)/RES))
        bx, by, bz = X//cells, Y//cells, Z//cells
        keys = (bx.astype(np.int64)<<40) | (by.astype(np.int64)<<20) | bz.astype(np.int64)
        order = np.argsort(keys, kind="stable")
        ks = keys[order]
        starts = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1]])
        for a, b in zip(starts, np.r_[starts[1:], len(ks)]):
            sel = order[a:b]
            yield (int(ks[a]), X[sel], Y[sel], Z[sel])

def main():
    d = np.loadtxt('/tmp/churn/arrivals.txt', dtype=np.int64)
    node, vox = d[:,0], d[:,1:]
    nodes_max = node.max()
    cuts = np.unique(np.linspace(0, nodes_max, UPDATES+1).astype(np.int64))[1:]
    schemes = ["global", "slab-8", "slab-32", "cube-8", "cube-4"]
    prev = {s: {} for s in schemes}
    hist = {s: [] for s in schemes}

    for ci, cut in enumerate(cuts):
        take = node <= cut
        V = vox[take]
        order = np.lexsort((V[:,2], V[:,1], V[:,0]))       # sort by x, then y, then z
        X, Y, Z = V[order,0], V[order,1], V[order,2]
        for s in schemes:
            dirty_bytes = dirty_units = total_units = 0
            seen = set()
            for key, ux, uy, uz in units_for(s, X, Y, Z):
                total_units += 1
                for axis, arr in (("x",ux),("y",uy),("z",uz)):
                    bands = haar_bands(arr)
                    unit_dirty = False
                    for bi, band in enumerate(bands):
                        fp = hash(band.tobytes())
                        k = (key, axis, bi)
                        seen.add(k)
                        if prev[s].get(k) != fp:
                            prev[s][k] = fp
                            dirty_bytes += varint_bytes(band)
                            unit_dirty = True
                    if unit_dirty: dirty_units += 1
            hist[s].append(dict(update=ci+1, voxels=int(len(X)),
                                dirty_kb=dirty_bytes/1024, dirty_units=dirty_units,
                                total_units=total_units))
        print(f"update {ci+1}/{len(cuts)}  voxels={len(X):,}  " +
              "  ".join(f"{s}={hist[s][-1]['dirty_kb']:.0f}KB" for s in schemes), flush=True)
    json.dump(hist, open('/tmp/churn/hist.json','w'))

if __name__ == '__main__':
    main()
