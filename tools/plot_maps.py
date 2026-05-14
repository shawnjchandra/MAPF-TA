#!/usr/bin/env python3
"""
Plot heatmap (GCM), voronoi map, highway, dan HPA gates dari file dump JSON.

Pakai:
    python3 tools/plot_maps.py <maps.json> [--out-dir OUT] [--what heatmap voronoi highway hpa]
                                          [--agg max|mean|sum|peak] [--dpi 200]
                                          [--cmap viridis] [--orient N]

Contoh:
    python3 tools/plot_maps.py output.maps.json
    python3 tools/plot_maps.py run/foo.maps.json --what heatmap --agg max --cmap inferno
    python3 tools/plot_maps.py run/foo.maps.json --what heatmap --orient 0  # arah East saja

Dependency: numpy, matplotlib (pip install numpy matplotlib).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import FancyArrowPatch


ORIENT_NAMES = {0: "East", 1: "South", 2: "West", 3: "North"}
# Vektor (drow, dcol) untuk setiap orientasi. Konsisten dgn getOrientationBetween di kode.
# Cek manual kalau hasilnya beda: orientasi 0 mungkin perlu disesuaikan.
ORIENT_DR_DC = {
    0: (0, 1),    # East  -> kolom +1
    1: (1, 0),    # South -> baris +1
    2: (0, -1),   # West  -> kolom -1
    3: (-1, 0),   # North -> baris -1
}


# ---------- I/O ----------
def load_maps(path: str) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def get_dims(data: dict) -> tuple[int, int]:
    m = data["meta"]
    return int(m["rows"]), int(m["cols"])


def loc_to_rc(loc: int, cols: int) -> tuple[int, int]:
    return loc // cols, loc % cols


def obstacle_mask(data: dict) -> np.ndarray:
    rows, cols = get_dims(data)
    obs = np.array(data["map"]["obstacles"], dtype=np.int8).reshape(rows, cols)
    return obs == 1  # True = obstacle


# ---------- Aggregations ----------
def gcm_to_grid(data: dict, agg: str, orient: int | None) -> np.ndarray:
    """Konversi GCM ke grid 2D (rows, cols).

    agg:
        'max'  : max ke-4 orientasi per cell  (default - paling intuitif untuk heatmap)
        'mean' : rata-rata ke-4 orientasi
        'sum'  : jumlah ke-4 orientasi
        'peak' : pakai w_peak_val (puncak yang pernah tercatat)
    orient: kalau bukan None (0..3), pakai orientasi tunggal itu dan abaikan agg.
    """
    rows, cols = get_dims(data)
    gcm = data["heatmap"]["gcm"]

    if not gcm:
        return np.full((rows, cols), np.nan)

    arr = np.array(gcm, dtype=float)  # shape (map_size, 4)
    if orient is not None:
        flat = arr[:, orient]
    elif agg == "peak":
        peak = data["heatmap"].get("w_peak_val", [])
        if not peak:
            print("[warn] w_peak_val kosong, fallback ke agg=max")
            flat = arr.max(axis=1)
        else:
            flat = np.array(peak, dtype=float)
    elif agg == "mean":
        flat = arr.mean(axis=1)
    elif agg == "sum":
        flat = arr.sum(axis=1)
    else:  # 'max' default
        flat = arr.max(axis=1)

    return flat.reshape(rows, cols)


# ---------- Plotters ----------
def plot_heatmap(data: dict, out_path: Path, *, agg: str = "max",
                 orient: int | None = None, cmap: str = "viridis",
                 dpi: int = 200, title: str | None = None):
    rows, cols = get_dims(data)
    grid = gcm_to_grid(data, agg, orient)
    obs = obstacle_mask(data)

    # Maskingin obstacle utk tidak ikut warna heatmap
    grid_masked = np.where(obs, np.nan, grid)

    fig, ax = plt.subplots(figsize=(max(5, cols / 8), max(5, rows / 8)))

    # Set bad (NaN) jadi hitam utk obstacle
    cm = plt.get_cmap(cmap).copy()
    cm.set_bad("black")

    # Skala minimum di baseline (1.0) supaya kontras
    vmin = max(1.0, float(np.nanmin(grid_masked)))
    vmax = float(np.nanmax(grid_masked)) if np.isfinite(np.nanmax(grid_masked)) else vmin + 1.0
    if vmax <= vmin:
        vmax = vmin + 1.0

    im = ax.imshow(grid_masked, cmap=cm, vmin=vmin, vmax=vmax,
                   interpolation="nearest", origin="upper")
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Bobot GCM" + (f" (orient {ORIENT_NAMES[orient]})" if orient is not None
                                  else f" (agg={agg})"))

    # Judul
    meta = data["meta"]
    if title is None:
        title = (f"Heatmap GCM — {meta.get('map_name','?')} | "
                 f"mode={meta.get('mode','?')} | "
                 f"agents={meta.get('num_of_agents','?')} | "
                 f"t={meta.get('final_timestep','?')}")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("col"); ax.set_ylabel("row")
    ax.set_xticks([]); ax.set_yticks([])

    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"[heatmap] -> {out_path}")


def plot_heatmap_all_orient(data: dict, out_path: Path, *, cmap: str = "viridis",
                             dpi: int = 200):
    """Plot 4 panel orient di satu figure (East/South/West/North)."""
    rows, cols = get_dims(data)
    obs = obstacle_mask(data)
    gcm = data["heatmap"]["gcm"]
    if not gcm:
        print("[heatmap_4] gcm kosong, skip")
        return
    arr = np.array(gcm, dtype=float)

    fig, axes = plt.subplots(2, 2, figsize=(max(10, cols / 5), max(10, rows / 5)))
    cm = plt.get_cmap(cmap).copy()
    cm.set_bad("black")

    vmin = max(1.0, float(arr.min()))
    vmax = float(arr.max())
    if vmax <= vmin: vmax = vmin + 1.0

    for o, ax in zip([0, 1, 2, 3], axes.flat):
        g = arr[:, o].reshape(rows, cols)
        g = np.where(obs, np.nan, g)
        im = ax.imshow(g, cmap=cm, vmin=vmin, vmax=vmax,
                       interpolation="nearest", origin="upper")
        ax.set_title(f"Orient {o} = {ORIENT_NAMES[o]}", fontsize=10)
        ax.set_xticks([]); ax.set_yticks([])

    fig.colorbar(im, ax=axes.ravel().tolist(), fraction=0.025, pad=0.03,
                 label="Bobot GCM per arah")
    meta = data["meta"]
    fig.suptitle(f"Heatmap GCM per Orientasi — {meta.get('map_name','?')}", fontsize=12)
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"[heatmap_4orient] -> {out_path}")


def _qualitative_cmap(n: int) -> ListedColormap:
    """Cmap kategori; pakai tab20 untuk n<=20, hsv untuk n>20."""
    if n <= 20:
        base = plt.get_cmap("tab20", 20)
        colors = [base(i % 20) for i in range(n)]
    else:
        base = plt.get_cmap("hsv", n)
        colors = [base(i) for i in range(n)]
    return ListedColormap(colors)


def plot_voronoi(data: dict, out_path: Path, *, dpi: int = 200):
    rows, cols = get_dims(data)
    vor = data["voronoi"]["voronoi_map"]
    k = int(data["voronoi"]["k"])
    if not vor:
        print("[voronoi] kosong, skip")
        return

    grid = np.array(vor, dtype=int).reshape(rows, cols)
    obs = obstacle_mask(data)

    # Buat cmap: cluster 0..k-1 kategori, -1 dan obstacle -> hitam
    cm = _qualitative_cmap(k)
    bounds = np.arange(-0.5, k + 0.5, 1.0)
    norm = BoundaryNorm(bounds, cm.N)

    disp = np.where((grid < 0) | obs, np.nan, grid.astype(float))

    fig, ax = plt.subplots(figsize=(max(5, cols / 8), max(5, rows / 8)))
    cm_with_bad = cm
    cm_with_bad.set_bad("black")
    im = ax.imshow(disp, cmap=cm_with_bad, norm=norm,
                   interpolation="nearest", origin="upper")
    cb = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, ticks=np.arange(k))
    cb.set_label(f"Cluster id (k={k})")

    meta = data["meta"]
    ax.set_title(f"Voronoi Partition — {meta.get('map_name','?')} | k={k}", fontsize=10)
    ax.set_xticks([]); ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"[voronoi] -> {out_path}")


def plot_highway(data: dict, out_path: Path, *, dpi: int = 200,
                 show_penalized: bool = False, max_edges: int = 5000):
    rows, cols = get_dims(data)
    obs = obstacle_mask(data)

    fig, ax = plt.subplots(figsize=(max(5, cols / 6), max(5, rows / 6)))

    # Background: peta obstacle abu-abu
    bg = np.where(obs, 1.0, 0.0)
    ax.imshow(bg, cmap=ListedColormap(["white", "#222222"]),
              interpolation="nearest", origin="upper")

    edges_allowed   = data["highway"].get("edges_allowed", [])
    edges_penalized = data["highway"].get("edges_penalized", [])

    def draw(edges, color, label):
        if not edges:
            return 0
        n = min(len(edges), max_edges)
        for from_loc, _from_or, to_loc, _to_or in edges[:n]:
            fr, fc = loc_to_rc(from_loc, cols)
            tr, tc = loc_to_rc(to_loc, cols)
            arr = FancyArrowPatch(
                (fc, fr), (tc, tr),
                arrowstyle="->", mutation_scale=8,
                color=color, alpha=0.85, linewidth=1.2,
            )
            ax.add_patch(arr)
        return n

    n_a = draw(edges_allowed, "#1f77b4", "allowed")
    n_p = 0
    if show_penalized:
        n_p = draw(edges_penalized, "#d62728", "penalized")

    # Legend kecil
    handles = [plt.Line2D([0], [0], color="#1f77b4", lw=2,
                          label=f"allowed ({n_a})")]
    if show_penalized:
        handles.append(plt.Line2D([0], [0], color="#d62728", lw=2,
                                  label=f"penalized ({n_p})"))
    ax.legend(handles=handles, loc="upper right", fontsize=8)

    meta = data["meta"]
    ax.set_title(f"Highway — {meta.get('map_name','?')} | c_penalty={meta.get('c_penalty','?')}",
                 fontsize=10)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)  # origin upper
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"[highway] -> {out_path}")


def plot_hpa(data: dict, out_path: Path, *, dpi: int = 200):
    """Gambar voronoi sebagai background + gate sebagai marker + edge antar
    cluster sebagai garis tipis (kalau cluster_adj tersedia)."""
    rows, cols = get_dims(data)
    obs = obstacle_mask(data)
    vor = data["voronoi"]["voronoi_map"]
    k   = int(data["voronoi"]["k"])
    gates = data["hpa"].get("gates", [])
    gates_per_cluster = data["hpa"].get("gates_per_cluster", [])
    cluster_adj = data["hpa"].get("cluster_adj", [])

    if not vor:
        print("[hpa] voronoi kosong, skip")
        return

    grid = np.array(vor, dtype=int).reshape(rows, cols)
    cm = _qualitative_cmap(k)
    cm.set_bad("black")
    disp = np.where((grid < 0) | obs, np.nan, grid.astype(float))

    fig, ax = plt.subplots(figsize=(max(6, cols / 6), max(6, rows / 6)))
    bounds = np.arange(-0.5, k + 0.5, 1.0)
    ax.imshow(disp, cmap=cm, norm=BoundaryNorm(bounds, cm.N),
              interpolation="nearest", origin="upper", alpha=0.55)

    # Centroid per cluster: kira-kira posisi rata-rata cell-cluster
    centroids_rc = []
    for c in range(k):
        idx = np.where(grid == c)
        if len(idx[0]) == 0:
            centroids_rc.append(None)
        else:
            centroids_rc.append((float(np.mean(idx[0])), float(np.mean(idx[1]))))

    # Garis cluster-adjacency
    if cluster_adj:
        for c_a, adj in enumerate(cluster_adj):
            ra = centroids_rc[c_a]
            if ra is None: continue
            for c_b, _w in adj:
                if c_b <= c_a: continue
                rb = centroids_rc[c_b]
                if rb is None: continue
                ax.plot([ra[1], rb[1]], [ra[0], rb[0]],
                        color="white", linewidth=0.8, alpha=0.9)

    # Gates
    if gates:
        gx, gy = [], []
        for loc in gates:
            r, c = loc_to_rc(loc, cols)
            gx.append(c); gy.append(r)
        ax.scatter(gx, gy, s=14, c="black", marker="o", edgecolors="white",
                   linewidths=0.5, label=f"gates ({len(gates)})", zorder=5)

    # Centroid marker
    for c, rc in enumerate(centroids_rc):
        if rc is None: continue
        ax.scatter([rc[1]], [rc[0]], s=70, c="white", marker="*",
                   edgecolors="black", linewidths=0.7, zorder=6)

    meta = data["meta"]
    ax.set_title(f"HPA Abstraction — {meta.get('map_name','?')} | "
                 f"clusters={k}, gates={len(gates)}", fontsize=10)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    if gates:
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"[hpa] -> {out_path}")


# ---------- CLI ----------
def main():
    ap = argparse.ArgumentParser(description="Render heatmap & teman-teman dari maps.json")
    ap.add_argument("maps_json", help="path ke <output>.maps.json hasil dump_maps")
    ap.add_argument("--out-dir", default=None,
                    help="folder output PNG (default: folder yang sama dengan maps_json)")
    ap.add_argument("--what", nargs="+",
                    default=["heatmap", "voronoi", "highway", "hpa"],
                    choices=["heatmap", "heatmap4", "voronoi", "highway", "hpa", "all"],
                    help="visualisasi yang dibuat")
    ap.add_argument("--agg", default="max", choices=["max", "mean", "sum", "peak"],
                    help="aggregasi 4 orientasi untuk heatmap (default: max)")
    ap.add_argument("--orient", type=int, default=None, choices=[0, 1, 2, 3],
                    help="kalau di-set, plot heatmap orientasi tunggal saja")
    ap.add_argument("--cmap", default="viridis", help="colormap heatmap (mpl name)")
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--show-penalized", action="store_true",
                    help="ikut gambar arah highway yang dipenalty (merah)")
    ap.add_argument("--max-edges", type=int, default=5000,
                    help="batas jumlah edges highway yang digambar")
    args = ap.parse_args()

    in_path = Path(args.maps_json)
    if not in_path.is_file():
        sys.exit(f"file tidak ada: {in_path}")
    out_dir = Path(args.out_dir) if args.out_dir else in_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = in_path.stem.replace(".maps", "")

    data = load_maps(str(in_path))

    what = set(args.what)
    if "all" in what:
        what = {"heatmap", "heatmap4", "voronoi", "highway", "hpa"}

    if "heatmap" in what:
        suffix = f"orient{args.orient}" if args.orient is not None else args.agg
        plot_heatmap(data, out_dir / f"{stem}.heatmap.{suffix}.png",
                     agg=args.agg, orient=args.orient,
                     cmap=args.cmap, dpi=args.dpi)
    if "heatmap4" in what:
        plot_heatmap_all_orient(data, out_dir / f"{stem}.heatmap.4orient.png",
                                cmap=args.cmap, dpi=args.dpi)
    if "voronoi" in what:
        plot_voronoi(data, out_dir / f"{stem}.voronoi.png", dpi=args.dpi)
    if "highway" in what:
        plot_highway(data, out_dir / f"{stem}.highway.png", dpi=args.dpi,
                     show_penalized=args.show_penalized,
                     max_edges=args.max_edges)
    if "hpa" in what:
        plot_hpa(data, out_dir / f"{stem}.hpa.png", dpi=args.dpi)


if __name__ == "__main__":
    main()