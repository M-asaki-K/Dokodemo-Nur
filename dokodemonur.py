# -*- coding: utf-8 -*-
"""
A→B ルーティング（Uターン不可＋右左折最小→距離最小・楕円ゲート版）:
- curvy 候補抽出（狭道除外=ソフト、unclassified も対象。高速系は候補から除外）
- AB 楕円ゲート＋percentile 自動降下で n 本を確保
- ラウンドアバウト袋小路（①）で端点をスクリーニング
- 多対多の“距離＋右左折数（辞書式）」で順序＆向き DP 最適化
- 同一エッジ再通行・Uターン修復（代替経路探索→forbid→必要ならcurvyドロップ）
- Folium（と任意で Matplotlib）で可視化

★本版の追加:
- curvy の entry/exit スナップを「本体ノード列にもクリップ反映」
- 反転時もクリップ再適用
- build_full_node_path をクリップ対応に変更

★ログ（デバッグ可視化）の追加:
- dprint() ヘルパと各種詳細ログ
"""

import re
import math
import random
import numpy as np
import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.ops import unary_union

# ========= 設定 =========
# A→B（例: パピヨンシャトー → 坂田植木）
ORIGIN_LATLON = (43.836419647725236, 142.53064142690536)
DEST_LATLON   = (43.75758012406924, 142.4437582873635)

# A→B（例: 自宅 → asahiyama zoo）
ORIGIN_LATLON = (43.730221548916596, 142.38388084900924)
DEST_LATLON   = (43.768802682478, 142.47555914795294)

CENTER_LATLON = ORIGIN_LATLON
SEARCH_RADIUS_M = 25000
NETWORK_TYPE = "drive"

# curvy 抽出
P_INIT, P_MIN, P_STEP = 90, 10, 5     # percentile 自動降下
REQUIRE_N = 5                         # 欲しい curvy 本数
MIN_LENGTH_M = 500.0
EXCLUDE_SHORT_INTERSECTION_LINKS = True
INTERSECTION_LINK_MAX_LEN_M = 40.0

# “狭そう”フィルタ（True=狭いと判定→curvy候補から除外）
NARROW_HIGHWAY_BLACKLIST = {
    "residential", "unclassified", "service", "living_street",
    "track", "path", "footway", "bridleway", "cycleway",
    "steps", "pedestrian"
}
WIDE_HIGHWAY_WHITELIST = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"
}
# 狭いタグでも“広い証拠”があれば救済
MIN_LANES_FOR_ESCAPE = 2
MIN_WIDTH_M_FOR_ESCAPE = 5.0
MIN_MAXSPEED_KMH_FOR_ESCAPE = 40

# 行き止まりcurvyを除外（①：ラウンドアバウト外への実用出口チェックのみ）
EXCLUDE_DEADEND_CURVY = True

# AB 楕円ゲート
ELLIPSE_USE_ALPHA = True
ELLIPSE_ALPHA = 0.35
ELLIPSE_SLACK_M = 8000.0

# --- コネクタを“ターン意識”で最短化する ---
TURN_AWARE_CONNECTOR = True
TURN_THRESH_DEG = 45.0        # 「右左折」と数える角度しきい値
UTURN_FORBID_DEG = 170.0      # これ以上の反転は遷移禁止
TURN_LEXICOGRAPHIC_WEIGHT = 1_000_000.0  # ターン1回の重み

# コネクタ探索の高速系扱い
EXCLUDE_HIGHWAY_CLASSES = {  # curvy 候補からは無条件で除外
    "motorway", "trunk", "motorway_link", "trunk_link",
}
CONNECTOR_FORBID_HIGHWAYS = True
CONNECTOR_HIGHWAY_PENALTY_M = {
    "motorway": 3000.0,
    "trunk":    2000.0,
    "primary":   800.0,
}

# 最適化対象
RANDOM_SEED = 42
ENABLE_DROP_CURVY_ON_FORCED_UTURN = True
MAX_CURVY_DROPS = 2

# 可視化
SAVE_FOLIUM_HTML = "ab_curvy_route_map.html"
PLOT_MATPLOTLIB = False
ZOOM_TO_ROUTE = True

# ノード列の出力/保存
PRINT_ROUTE_NODE_IDS   = True
SAVE_ROUTE_NODE_CSV    = "route_nodes.csv"
SAVE_ROUTE_NODE_GEOJSON= "route_nodes.geojson"

# ルートのループ消去
ENABLE_LOOP_ERASE = True
LOOP_ERASE_MODE  = "full"         # "pingpong" or "full"

# 直近に生成した folium.Map を保持
LAST_FOLIUM_MAP = None

# ファイル先頭の設定群のすぐ下あたりに追加
DISCOVERY_CTX = None  # {"G_multi":..., "G":..., "orig":..., "dest":..., "candidates": [...]}

# ========= デバッグ =========
DEBUG = True           # ログを止めたい時は False
DEBUG_MAX_EDGE_SAMPLES = 5

def dprint(*args, **kw):
    if DEBUG:
        print(*args, **kw)

def _fmt_edges(es):
    es = list(es)
    if not es:
        return "[]"
    head = es[:DEBUG_MAX_EDGE_SAMPLES]
    rest = "" if len(es) <= DEBUG_MAX_EDGE_SAMPLES else f" ...(+{len(es)-DEBUG_MAX_EDGE_SAMPLES})"
    return "[" + ", ".join(f"{a}-{b}" for a, b in head) + "]" + rest

def _bearing_pair_info(G, prev_nodes, next_nodes):
    try:
        if not prev_nodes or not next_nodes:
            return None
        if len(prev_nodes) < 2 or len(next_nodes) < 2:
            return None
        if prev_nodes[-1] != next_nodes[0]:
            return None
        b1 = _bearing_uv(G, prev_nodes[-2], prev_nodes[-1])
        b2 = _bearing_uv(G, next_nodes[0], next_nodes[1])
        ang = _angle_diff_deg(b1, b2)
        return b1, b2, ang
    except Exception:
        return None

# ========= 設定 =========
ORDER_MODE = "ab_linear"  # "dp" | "ab_linear"

# ========= ユーティリティ =========
def _parse_number(s):
    if s is None: return None
    if isinstance(s, (int, float)): return float(s)
    m = re.search(r"[-+]?\d+(\.\d+)?", str(s))
    return float(m.group(0)) if m else None

def _to_kmh(v):
    if v is None: return None
    if isinstance(v, (int, float)): return float(v)
    s = str(v).lower()
    num = _parse_number(s)
    if num is None: return None
    if "mph" in s: return num * 1.60934
    return num

def _normalize_highways(value):
    if value is None: return []
    if isinstance(value, (list, tuple, set)): return [str(x) for x in value]
    return [str(value)]

def looks_narrow(edge: dict) -> bool:
    """True→狭そう（= curvy 候補から除外）"""
    hwy = edge.get("highway")
    def narrow_base(h): return h in NARROW_HIGHWAY_BLACKLIST
    def wide_base(h):   return h in WIDE_HIGHWAY_WHITELIST
    if isinstance(hwy, (list, tuple, set)):
        hset = set(map(str, hwy))
        if hset & WIDE_HIGHWAY_WHITELIST:
            return False
        if hset & NARROW_HIGHWAY_BLACKLIST:
            lanes = _parse_number(edge.get("lanes"))
            width_m = _parse_number(edge.get("width"))
            maxspeed = _to_kmh(edge.get("maxspeed"))
            if (lanes and lanes >= MIN_LANES_FOR_ESCAPE) or \
               (width_m and width_m >= MIN_WIDTH_M_FOR_ESCAPE) or \
               (maxspeed and maxspeed >= MIN_MAXSPEED_KMH_FOR_ESCAPE):
                return False
            return True
        return False
    else:
        if hwy is None: return False
        hwy = str(hwy)
        if wide_base(hwy): return False
        if narrow_base(hwy):
            lanes = _parse_number(edge.get("lanes"))
            width_m = _parse_number(edge.get("width"))
            maxspeed = _to_kmh(edge.get("maxspeed"))
            if (lanes and lanes >= MIN_LANES_FOR_ESCAPE) or \
               (width_m and width_m >= MIN_WIDTH_M_FOR_ESCAPE) or \
               (maxspeed and maxspeed >= MIN_MAXSPEED_KMH_FOR_ESCAPE):
                return False
            return True
        return False

def is_intersection_node(G, n) -> bool:
    d = G.nodes[n]
    sc = d.get("street_count")
    if sc is not None: return sc >= 3
    return G.degree(n) >= 3

def gc_m(lat1, lon1, lat2, lon2):
    return float(ox.distance.great_circle_vec(lat1, lon1, lat2, lon2))

def segment_curvature(edge: dict) -> float:
    geom = edge.get("geometry")
    if not isinstance(geom, LineString): return 0.0
    length = geom.length
    (x0, y0) = geom.coords[0]
    (x1, y1) = geom.coords[-1]
    straight = math.hypot(x1 - x0, y1 - y0)
    if straight <= 0: return 0.0
    return max(length / straight - 1.0, 0.0)

def edge_has_highway_in(edge, classes: set) -> bool:
    h = edge.get("highway")
    if h is None: return False
    if isinstance(h, (list, tuple, set)):
        return any(str(x) in classes for x in h)
    return str(h) in classes

def highway_penalty_for_edge(edge, penalty_map: dict) -> float:
    h = edge.get("highway")
    if h is None: return 0.0
    if isinstance(h, (list, tuple, set)):
        return max((penalty_map.get(str(x), 0.0) for x in h), default=0.0)
    return float(penalty_map.get(str(h), 0.0))

def sum_original_length_on_path(G, path):
    L = 0.0
    for u, v in zip(path[:-1], path[1:]):
        data = G.get_edge_data(u, v) or G.get_edge_data(v, u)
        if not data:
            x0, y0 = G.nodes[u]["x"], G.nodes[u]["y"]
            x1, y1 = G.nodes[v]["x"], G.nodes[v]["y"]
            L += ((x1 - x0)**2 + (y1 - y0)**2) ** 0.5
            continue
        if isinstance(data, dict) and any(isinstance(d, dict) for d in data.values()):
            L += float(min(d.get("length", 0.0) for d in data.values()))
        else:
            L += float(data.get("length", 0.0))
    return L

def sum_cost_len_on_path(G, path):
    S = 0.0
    for u, v in zip(path[:-1], path[1:]):
        data = G.get_edge_data(u, v)
        if not data:
            x0, y0 = G.nodes[u]["x"], G.nodes[u]["y"]
            x1, y1 = G.nodes[v]["x"], G.nodes[v]["y"]
            S += ((x1 - x0)**2 + (y1 - y0)**2) ** 0.5
        else:
            S += float(data.get("cost_len", data.get("length", 0.0)))
    return S

def bearing_between(G, a, b):
    ax, ay = G.nodes[a]["x"], G.nodes[a]["y"]
    bx, by = G.nodes[b]["x"], G.nodes[b]["y"]
    ang = math.degrees(math.atan2(bx - ax, by - ay))
    return ang + 360.0 if ang < 0 else ang

def count_turns_on_path(G, path, thresh_deg=50.0):
    if not path or len(path) < 3: return 0
    turns = 0
    prev_bear = bearing_between(G, path[0], path[1])
    for i in range(1, len(path) - 1):
        cur_bear = bearing_between(G, path[i], path[i + 1])
        d = abs(cur_bear - prev_bear)
        d = min(d, 360.0 - d)
        if d >= thresh_deg: turns += 1
        prev_bear = cur_bear
    return turns

def _bearing_uv(G, u, v):
    ax, ay = G.nodes[u]["x"], G.nodes[u]["y"]
    bx, by = G.nodes[v]["x"], G.nodes[v]["y"]
    ang = math.degrees(math.atan2(bx - ax, by - ay))
    return ang + 360.0 if ang < 0 else ang

def _angle_diff_deg(a, b):
    d = abs(a - b)
    return d if d <= 180.0 else 360.0 - d

def is_uturn_between_segments(G, prev_nodes, next_nodes, thresh_deg=170.0):
    if not prev_nodes or not next_nodes: return False
    if len(prev_nodes) < 2 or len(next_nodes) < 2: return False
    if prev_nodes[-1] != next_nodes[0]: return False
    b1 = _bearing_uv(G, prev_nodes[-2], prev_nodes[-1])
    b2 = _bearing_uv(G, next_nodes[0], next_nodes[1])
    return _angle_diff_deg(b1, b2) >= thresh_deg

def edge_set_from_node_path(path):
    if not path or len(path) < 2: return frozenset()
    edges = []
    for u, v in zip(path[:-1], path[1:]):
        a, b = (u, v) if u < v else (v, u)
        edges.append((a, b))
    return frozenset(edges)

def print_route_nodes(G_multi, nodes):
    """順番・ID・緯度・経度をタブ区切りで標準出力"""
    print("\n=== ROUTE NODES (index\tid\tlat\tlon) ===")
    for i, n in enumerate(nodes):
        lat = G_multi.nodes[n]["y"]; lon = G_multi.nodes[n]["x"]
        print(f"{i}\t{n}\t{lat:.6f}\t{lon:.6f}")

def save_route_nodes_csv(G_multi, nodes, path):
    import csv
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["index", "node_id", "lat", "lon"])
        for i, n in enumerate(nodes):
            w.writerow([i, n, G_multi.nodes[n]["y"], G_multi.nodes[n]["x"]])
    print(f"Saved node list CSV: {path}")

def save_route_nodes_geojson(G_multi, nodes, path):
    """ノードを Point の FeatureCollection に保存（順序はproperties.indexに格納）"""
    import json
    feats = []
    for i, n in enumerate(nodes):
        lat = G_multi.nodes[n]["y"]; lon = G_multi.nodes[n]["x"]
        feats.append({
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [lon, lat]},
            "properties": {"index": i, "node_id": int(n)}
        })
    fc = {"type": "FeatureCollection", "features": feats}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(fc, f, ensure_ascii=False)
    print(f"Saved node list GeoJSON: {path}")

def loop_erase_pingpong(nodes):
    """A,B,A の往復だけ素早く除去（スタック1行）"""
    out = []
    for n in nodes:
        if len(out) >= 2 and n == out[-2]:
            out.pop()  # 中間のBを捨てる
        else:
            out.append(n)
    return out

def loop_erase_full(nodes):
    """
    一般のループ消去（同一ノードが再出現したら、その間を丸ごと切る）
    Loop-Erased Path（LERW と同じ要領）
    """
    pos = {}   # node -> index in 'out'
    out = []
    for n in nodes:
        if n in pos:
            i = pos[n]
            for m in out[i+1:]:
                pos.pop(m, None)
            out = out[:i+1]
        else:
            pos[n] = len(out)
            out.append(n)
    return out

def _first_duplicate_idx(nodes):
    """最初に重複したノードを返す: (dup_idx, first_idx) / ない場合 None"""
    seen = {}
    for i, n in enumerate(nodes):
        if n in seen:
            return i, seen[n]
        seen[n] = i
    return None

def _copy_graph_without(G, banned_nodes, banned_undirected, keep_node=None):
    """禁止ノード/無向エッジを物理削除した DiGraph を作る（keep_node は残す）"""
    G2 = G.copy()
    # ノード削除
    for n in set(banned_nodes or []) - ({keep_node} if keep_node is not None else set()):
        if n in G2: G2.remove_node(n)
    # 無向エッジ削除（両方向とも落とす）
    for a, b in set(banned_undirected or []):
        if G2.has_edge(a, b): G2.remove_edge(a, b)
        if G2.has_edge(b, a): G2.remove_edge(b, a)
    return G2

# ========= クリップ対応の curvy ノード列取得 =========
def curvy_nodes_in_direction(route, dir_flag):
    """従来版：dir_flag=0: u->v, 1: v->u"""
    return route["nodes"] if dir_flag == 0 else list(reversed(route["nodes"]))

def curvy_nodes_dir_clipped(route, dir_flag, clip):
    """
    dir_flag: 0=u->v, 1=v->u
    clip: None または (entry_node, exit_node)
    """
    nodes = route["nodes"] if dir_flag == 0 else list(reversed(route["nodes"]))
    if not clip:
        return nodes
    e_in, e_out = clip
    try:
        i0 = nodes.index(e_in)
    except ValueError:
        i0 = 0
    try:
        i1 = nodes.index(e_out)
    except ValueError:
        i1 = len(nodes) - 1
    if i0 > i1:
        i0, i1 = i1, i0
    return nodes[i0:i1+1]

def salvage_reroute_from_prefix_once(G, G_multi, orig_node, dest_node,
                                     routes_sel, sequence, path_lookup,
                                     passed_route_ids_prefix,
                                     curvy_clip=None):
    """
    いま得られた sequence/path_lookup から full_nodes を合成し、
    最初の重複で手前までを確定 → そこから先を『禁止ノード/エッジで封鎖』して引き直す。
    成功すれば (new_full_nodes, new_used_ids, True) を返す。
    重複がなければ (full_nodes, used_ids, False)。
    引き直し不可なら None を返す。
    ※ 本版は curvy_clip に対応
    """
    curvy_clip = curvy_clip or {}

    nodes, used_route_ids = [], []
    node_pos = {}
    passed_ids = set(passed_route_ids_prefix or [])

    def append_seg(seg, cur_kind=None, cur_rid=None):
        nonlocal nodes, node_pos, used_route_ids, passed_ids
        if not seg: return None
        it = seg if not nodes else (seg[1:] if nodes[-1] == seg[0] else seg)
        for j, n in enumerate(it):
            if n in node_pos:
                dup_idx, first_idx = len(nodes) + j, node_pos[n]
                dprint(f"[dup] node={n} first_at={first_idx} now={dup_idx} → salvage reroute")
                return ("dup", dup_idx, first_idx)
            node_pos[n] = len(nodes) + j
        nodes += it
        if cur_kind == "curvy" and cur_rid is not None:
            used_route_ids.append(cur_rid)
            passed_ids.add(cur_rid)
        return None

    cut_info = None
    for kind, info in sequence:
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info; seg = path_lookup.get((a, b))
            if seg is None: return None
            cut_info = append_seg(seg, cur_kind=kind)
        elif kind == "curvy":
            i, dir_flag = info; r = routes_sel[i]
            seg = curvy_nodes_dir_clipped(r, dir_flag, curvy_clip.get(i))
            cut_info = append_seg(seg, cur_kind="curvy", cur_rid=r["route_id"])
        if cut_info: break

    if not cut_info:
        return nodes, used_route_ids, False

    _, dup_idx, first_idx = cut_info
    start_mid_idx = max(0, first_idx)
    prefix_nodes = nodes[: start_mid_idx + 1]
    start_mid = prefix_nodes[-1]

    banned_nodes = set(prefix_nodes)
    banned_edges = edge_set_from_node_path(prefix_nodes)
    dprint(f"[salvage] prefix_len={len(prefix_nodes)} start_mid={start_mid} "
           f"banned_nodes={len(banned_nodes)} banned_edges={len(banned_edges)}")

    remaining = [r for r in routes_sel if r["route_id"] not in passed_ids]

    G2 = _copy_graph_without(G, banned_nodes, banned_edges, keep_node=start_mid)
    if len(G2) == 0 or (start_mid not in G2) or (dest_node not in G2):
        return None

    terminals = [start_mid, dest_node] + [n for r in remaining for n in (r["u"], r["v"])]
    terminals = list(dict.fromkeys([t for t in terminals if t in G2]))

    if TURN_AWARE_CONNECTOR:
        distL, turnsL, costL, pathL = multi_shortest_between_turnaware(
            G2, terminals,
            turn_thresh=TURN_THRESH_DEG,
            uturn_forbid=UTURN_FORBID_DEG,
            W=TURN_LEXICOGRAPHIC_WEIGHT
        )
    else:
        distL, turnsL, costL, pathL = multi_shortest_between_with_turns(G2, terminals)

    best_tuple, seq2 = optimize_order_orientation(remaining, start_mid, dest_node, costL, turnsL)
    if seq2 is None:
        dprint("[salvage] DP failed on reduced graph")
        return None

    tail_nodes, tail_used = build_full_node_path(seq2, remaining, pathL, curvy_clip=None)
    if tail_nodes is None or len(tail_nodes) < 2:
        return None
    new_full = prefix_nodes + tail_nodes[1:]
    new_used = used_route_ids + [rid for rid, _dir in (
        (remaining[i]["route_id"], info[1]) for kind, info in seq2 if kind == "curvy"
    )]
    dprint(f"[salvage] succeeded; new_tail_len={len(tail_nodes)}")
    return new_full, new_used, True

def _gc_from_node(G_multi, n, latlon):
    return gc_m(G_multi.nodes[n]["y"], G_multi.nodes[n]["x"], latlon[0], latlon[1])

def order_curvy_by_AtoB(G_multi, routes_sel, A_latlon, B_latlon):
    """
    A に近い端点→B に近い端点へと並べる。
    各 curvy の向きは「A に近い端から入り、遠い端に抜ける」。
    戻り: [(route_index, dir_flag, entry_node, exit_node), ...] 昇順
    """
    items = []
    for i, r in enumerate(routes_sel):
        du = _gc_from_node(G_multi, r["u"], A_latlon)
        dv = _gc_from_node(G_multi, r["v"], A_latlon)
        # 位置スカラー：Aからの近さ。小さい方が“早く通る”
        pos = min(du, dv)
        # 向き：Aに近い側をエントリにする
        if du <= dv:
            dir_flag, entry, exitn = 0, r["u"], r["v"]   # u->v
        else:
            dir_flag, entry, exitn = 1, r["v"], r["u"]   # v->u
        items.append((pos, i, dir_flag, entry, exitn))
    items.sort(key=lambda x: x[0])
    return [(i, dir_flag, entry, exitn) for _, i, dir_flag, entry, exitn in items]

def build_sequence_ab_linear(G_multi, routes_sel, orig, dest, A_latlon, B_latlon):
    """
    並べ替えと初期向きが決まった状態から、DPを使わずにシーケンスを構築。
    """
    plan = order_curvy_by_AtoB(G_multi, routes_sel, A_latlon, B_latlon)
    sequence = []
    # A → 最初のcurvy入口
    first_i, first_dir, first_entry, first_exit = plan[0]
    sequence.append(("conn_from_A", (orig, first_entry)))
    sequence.append(("curvy", (first_i, first_dir)))
    prev_exit = first_exit

    for (i, dir_flag, entry, exitn) in plan[1:]:
        sequence.append(("conn", (prev_exit, entry)))
        sequence.append(("curvy", (i, dir_flag)))
        prev_exit = exitn

    # 最後の curvy 出口 → B
    sequence.append(("conn_to_B", (prev_exit, dest)))
    return sequence


# --- 角度・端点候補ユーティリティ ------------------------------
def first_edge_bearing_of_path(G, nodes):
    if not nodes or len(nodes) < 2:
        return None
    return _bearing_uv(G, nodes[0], nodes[1])

def last_edge_bearing_of_path(G, nodes):
    if not nodes or len(nodes) < 2:
        return None
    return _bearing_uv(G, nodes[-2], G, nodes[-1])  # (bug) fixed below

# 修正: 上の関数のタイポを正す
def last_edge_bearing_of_path(G, nodes):
    if not nodes or len(nodes) < 2:
        return None
    return _bearing_uv(G, nodes[-2], nodes[-1])

def end_candidates(route, as_entry=True):
    """
    端点スナップ緩和:
      entry なら [u, nodes[1]]（長さ>=3の時）
      exit なら  [v, nodes[-2]]
    """
    nodes = route["nodes"]
    if len(nodes) >= 3:
        return ([route["u"], nodes[1]] if as_entry else [route["v"], nodes[-2]])
    return ([route["u"]] if as_entry else [route["v"]])

def curvy_nodes_in_direction(route, dir_flag):
    return route["nodes"] if dir_flag == 0 else list(reversed(route["nodes"]))

def bearing_from_node_step(G, seq, start_idx):
    if start_idx < 0 or start_idx+1 >= len(seq):
        return None
    return _bearing_uv(G, seq[start_idx], seq[start_idx+1])

def replace_tuple_in_sequence(sequence, idx, new_tuple):
    kind, _ = sequence[idx]
    sequence[idx] = (kind, new_tuple)

# ========= ターン最小＋Uターン禁止のコネクタ最短路 =========
def _build_arc_tables(G):
    arcs = []
    out_by_node = {n: [] for n in G.nodes}
    in_by_node  = {n: [] for n in G.nodes}
    base_cost = []
    bearing  = []
    for u, v, d in G.edges(data=True):
        aid = len(arcs)
        arcs.append((u, v))
        out_by_node[u].append(aid)
        in_by_node[v].append(aid)
        base_cost.append(float(d.get("cost_len", d.get("length", 1.0))))
        bearing.append(_bearing_uv(G, u, v))
    return arcs, out_by_node, in_by_node, base_cost, bearing

def _dijkstra_turnaware_from(G, tables, source,
                             turn_thresh=45.0, uturn_forbid=170.0, W=1e6):
    arcs, out_by_node, in_by_node, base_cost, bearing = tables
    import heapq
    INF = float("inf")
    dist = [INF] * len(arcs)
    prev = [None] * len(arcs)
    hq = []

    for aid in out_by_node.get(source, []):
        dist[aid] = base_cost[aid]
        prev[aid] = -1
        heapq.heappush(hq, (dist[aid], aid))

    while hq:
        dcur, aid = heapq.heappop(hq)
        if dcur != dist[aid]:
            continue
        u, v = arcs[aid]
        b1 = bearing[aid]
        for bid in out_by_node.get(v, []):
            x, w = arcs[bid]
            if w == u:
                continue
            b2 = bearing[bid]
            ang = _angle_diff_deg(b1, b2)
            if ang >= uturn_forbid:
                continue
            step = base_cost[bid] + (W if ang >= turn_thresh else 0.0)
            nd = dcur + step
            if nd < dist[bid]:
                dist[bid] = nd
                prev[bid] = aid
                heapq.heappush(hq, (nd, bid))
    return dist, prev

def _reconstruct_path_nodes_from_arc(prev, best_aid, arcs):
    if best_aid is None:
        return None
    seq = []
    aid = best_aid
    while aid is not None and aid != -1:
        seq.append(aid)
        aid = prev[aid]
    seq.reverse()
    if not seq:
        return None
    nodes = [arcs[seq[0]][0]]
    for aid in seq:
        nodes.append(arcs[aid][1])
    return nodes

def multi_shortest_between_turnaware(G, points,
                                     turn_thresh=45.0,
                                     uturn_forbid=170.0,
                                     W=1_000_000.0):
    arcs, out_by_node, in_by_node, base_cost, bearing = _build_arc_tables(G)
    tables = (arcs, out_by_node, in_by_node, base_cost, bearing)

    distL, turnsL, costL, pathL = {}, {}, {}, {}

    for s in points:
        dist_arc, prev_arc = _dijkstra_turnaware_from(
            G, tables, s, turn_thresh=turn_thresh, uturn_forbid=uturn_forbid, W=W
        )
        for t in points:
            if s == t:
                distL[(s, t)] = 0.0; turnsL[(s, t)] = 0; costL[(s, t)] = 0.0; pathL[(s, t)] = [s]
                continue
            best_aid, best_val = None, float("inf")
            for aid in in_by_node.get(t, []):
                if dist_arc[aid] < best_val:
                    best_val, best_aid = dist_arc[aid], aid
            if best_aid is None or not (best_val < float("inf")):
                distL[(s, t)] = turnsL[(s, t)] = costL[(s, t)] = pathL[(s, t)] = None
                continue
            nodes = _reconstruct_path_nodes_from_arc(prev_arc, best_aid, arcs)
            pathL[(s, t)] = nodes
            distL[(s, t)]  = sum_original_length_on_path(G, nodes)
            costL[(s, t)]  = sum_cost_len_on_path(G, nodes)
            turnsL[(s, t)] = count_turns_on_path(G, nodes, thresh_deg=turn_thresh)
    return distL, turnsL, costL, pathL

# ========= curvy 抽出・分解 =========
def collect_curvy_edges(G_multi, min_length_m=30.0,
                        exclude_short_intersection_links=True,
                        inter_link_max_len_m=40.0,
                        drop_narrow=True):
    segs = []
    intersections = set()
    if exclude_short_intersection_links:
        for n in G_multi.nodes:
            if is_intersection_node(G_multi, n):
                intersections.add(n)

    for u, v, k, data in G_multi.edges(keys=True, data=True):
        length = float(data.get("length", 0.0))
        if length < min_length_m:
            continue
        geom = data.get("geometry")
        if not isinstance(geom, LineString):
            continue
        if exclude_short_intersection_links and (u in intersections) and (v in intersections) and (length <= inter_link_max_len_m):
            continue
        if drop_narrow and looks_narrow(data):
            continue
        if EXCLUDE_HIGHWAY_CLASSES and edge_has_highway_in(data, EXCLUDE_HIGHWAY_CLASSES):
            continue

        curv = segment_curvature(data)
        hwys = tuple(sorted(set(_normalize_highways(data.get("highway")))))
        segs.append({"u": u, "v": v, "k": k, "geom": geom, "length": length, "curv": curv, "highways": hwys})
    return segs

def pick_curvy(segs, percentile=75, absolute_threshold=None):
    curvs = np.array([s["curv"] for s in segs], dtype=float)
    if len(curvs) == 0: return [], 0.0
    thr = float(np.percentile(curvs, percentile)) if absolute_threshold is None else float(absolute_threshold)
    return [s for s in segs if s["curv"] >= thr], thr

def build_curvy_subgraph(G_multi, picks):
    H = nx.Graph()
    for s in picks:
        u, v = s["u"], s["v"]
        geom = s["geom"]
        L    = float(s["length"])
        curv = float(s["curv"])
        hwys = set(s.get("highways", []))
        if H.has_edge(u, v):
            if L > H[u][v]["length"]:
                H[u][v].update(geometry=geom, length=L, curvature=curv)
            H[u][v]["highways"] |= hwys
        else:
            H.add_edge(u, v, geometry=geom, length=L, curvature=curv, highways=set(hwys))
    for n, d in G_multi.nodes(data=True):
        if n in H:
            H.nodes[n]["x"] = d.get("x")
            H.nodes[n]["y"] = d.get("y")
            H.nodes[n]["street_count"] = d.get("street_count")
    return H

def decompose_into_routes(H):
    from collections import Counter
    routes = []
    visited = set()

    def neighbors_unvisited(u):
        out = []
        for v in H.neighbors(u):
            ekey = (min(u, v), max(u, v))
            if ekey not in visited:
                out.append(v)
        return out
    def mark_edge(u, v):
        visited.add((min(u, v), max(u, v)))

    rid = 0
    for comp in nx.connected_components(H):
        Hc = H.subgraph(comp).copy()
        deg = dict(Hc.degree())
        terminals = [n for n, d in deg.items() if d == 1]

        def walk(start, prev=None):
            path = [start]
            cur = start
            while True:
                nbrs = neighbors_unvisited(cur)
                if prev is not None and prev in nbrs: nbrs.remove(prev)
                if not nbrs: break
                nxt = nbrs[0]
                mark_edge(cur, nxt)
                path.append(nxt)
                prev, cur = cur, nxt
                if Hc.degree(cur) != 2: break
            return path

        for s in terminals:
            if not neighbors_unvisited(s): continue
            path = walk(s, prev=None)
            if len(path) >= 2:
                rid += 1
                geom_list, Ls, curvs = [], [], []
                hw_counter = Counter()
                for a, b in zip(path[:-1], path[1:]):
                    ed = H.get_edge_data(a, b)
                    geom_list.append(ed["geometry"])
                    Ls.append(ed["length"])
                    curvs.append(ed["curvature"])
                    hw_counter.update(ed.get("highways", []))
                geom = unary_union(geom_list)
                highway_top = None
                highway_counts = {}
                if hw_counter:
                    highway_counts = dict(hw_counter.most_common())
                    highway_top = next(iter(hw_counter.most_common(1)))[0]
                routes.append({
                    "route_id": rid, "u": path[0], "v": path[-1], "nodes": path,
                    "length_m": float(sum(Ls)), "curv_mean": float(np.mean(curvs)) if curvs else 0.0,
                    "geometry": geom, "highway_top": highway_top, "highway_counts": highway_counts
                })
    return routes

# ========= ① ラウンドアバウト袋小路フィルタ =========
def is_roundabout_edge(edge_data: dict) -> bool:
    j = edge_data.get("junction")
    if j is None: return False
    if isinstance(j, (list, tuple, set)):
        return any(str(x).lower() == "roundabout" for x in j)
    return str(j).lower() == "roundabout"

def has_non_roundabout_exit(G_multi, node, incoming_neighbor) -> bool:
    for w in G_multi.neighbors(node):
        if w == incoming_neighbor: continue
        eds = G_multi.get_edge_data(node, w, default={}).values()
        if eds and all(is_roundabout_edge(ed) for ed in eds):
            continue
        return True
    return False

def both_endpoints_roundabout_safe(G_multi, route) -> bool:
    u, v = route["u"], route["v"]
    nodes = route.get("nodes")
    if not nodes or len(nodes) < 2: return False
    u_in, v_in = nodes[1], nodes[-2]
    def ok(node, incoming_neighbor):
        sc = G_multi.nodes[node].get("street_count", None)
        if not ((sc is None) or (sc >= 2)):
            return False
        return has_non_roundabout_exit(G_multi, node, incoming_neighbor)
    return ok(u, u_in) and ok(v, v_in)

# ========= AB 楕円ゲート =========
def ellipse_focus_sum_limit_m(A_latlon, B_latlon):
    ab = gc_m(A_latlon[0], A_latlon[1], B_latlon[0], B_latlon[1])
    if ELLIPSE_USE_ALPHA:
        return ab * (1.0 + ELLIPSE_ALPHA)
    return ab + ELLIPSE_SLACK_M

def geom_inside_AB_ellipse(geom, A_latlon, B_latlon, F_m) -> bool:
    p = geom.representative_point()
    lat, lon = p.y, p.x
    dA = gc_m(lat, lon, A_latlon[0], A_latlon[1])
    dB = gc_m(lat, lon, B_latlon[0], B_latlon[1])
    return (dA + dB) <= F_m

def filter_curvy_by_AB_ellipse(routes, A_latlon, B_latlon):
    F_m = ellipse_focus_sum_limit_m(A_latlon, B_latlon)
    return [r for r in routes if r.get("geometry") and geom_inside_AB_ellipse(r["geometry"], A_latlon, B_latlon, F_m)]

# ========= 多対多 “距離+右左折” =========
def multi_shortest_between_with_turns(G, points):
    dist, turns, cost, path = {}, {}, {}, {}
    for s in points:
        costs, paths = nx.single_source_dijkstra(G, s, weight="cost_len")
        for t in points:
            if s == t:
                dlen, k, c, p = 0.0, 0, 0.0, [s]
            else:
                if t not in costs:
                    dist[(s, t)] = turns[(s, t)] = cost[(s, t)] = path[(s, t)] = None
                    continue
                c, p = float(costs[t]), paths[t]
                k = count_turns_on_path(G, p, TURN_THRESH_DEG)
                dlen = sum_original_length_on_path(G, p)
            dist[(s, t)] = dlen; turns[(s, t)] = k; cost[(s, t)] = c; path[(s, t)] = p
    return dist, turns, cost, path

# ========= DP：順序＋向きの最適化 =========
def optimize_order_orientation(routes_sel, orig, dest, cost_lookup, turns_lookup):
    n = len(routes_sel)
    if n == 0: return None, None
    ends = [(r["u"], r["v"]) for r in routes_sel]
    Ls   = [float(r["length_m"]) for r in routes_sel]  # curvy距離は生で加算
    INF = 1e18
    size = 1 << n
    dp_cost = [[[(INF, INF)] * 2 for _ in range(n)] for _ in range(size)]
    dp_prev = [[[None] * 2 for _ in range(n)] for _ in range(size)]
    def better(a, b): return (a[0] < b[0]) or (a[0] == b[0] and a[1] < b[1])

    # 初期：A→i 入り
    for i in range(n):
        for ein in (0, 1):
            entry_node = ends[i][ein]
            cA = cost_lookup.get((orig, entry_node)); tA = turns_lookup.get((orig, entry_node))
            if cA is None or tA is None: continue
            cost_tuple = (int(tA), float(cA) + Ls[i])
            mask = 1 << i; eout = 1 - ein
            if better(cost_tuple, dp_cost[mask][i][eout]):
                dp_cost[mask][i][eout] = cost_tuple
                dp_prev[mask][i][eout] = (0, -1, -1, ein)

    # 遷移
    for mask in range(size):
        for i in range(n):
            for eout in (0, 1):
                cur_tuple = dp_cost[mask][i][eout]
                if cur_tuple[0] >= INF: continue
                cur_node = ends[i][eout]
                for k in range(n):
                    if mask & (1 << k): continue
                    for ein in (0, 1):
                        entry_node = ends[k][ein]
                        c = cost_lookup.get((cur_node, entry_node)); t = turns_lookup.get((cur_node, entry_node))
                        if c is None or t is None: continue
                        new_tuple = (cur_tuple[0] + int(t), cur_tuple[1] + float(c) + Ls[k])
                        new_mask = mask | (1 << k); new_eout = 1 - ein
                        if better(new_tuple, dp_cost[new_mask][k][new_eout]):
                            dp_cost[new_mask][k][new_eout] = new_tuple
                            dp_prev[new_mask][k][new_eout] = (mask, i, eout, ein)

    # 終了：最後の出口端 → B
    best_tuple, best_i, best_eout = (INF, INF), None, None
    full = size - 1
    for i in range(n):
        for eout in (0, 1):
            cur_tuple = dp_cost[full][i][eout]
            if cur_tuple[0] >= INF: continue
            exit_node = ends[i][eout]
            cB = cost_lookup.get((exit_node, dest)); tB = turns_lookup.get((exit_node, dest))
            if cB is None or tB is None: continue
            total_tuple = (cur_tuple[0] + int(tB), cur_tuple[1] + float(cB))
            if (total_tuple[0] < best_tuple[0]) or (total_tuple[0] == best_tuple[0] and total_tuple[1] < best_tuple[1]):
                best_tuple, best_i, best_eout = total_tuple, i, eout
    if best_i is None: return None, None

    # 復元
    i, eout, mask = best_i, best_eout, full
    sequence = []
    sequence.append(("conn_to_B", (ends[i][eout], dest)))
    while mask:
        pmask, pi, peout, ein = dp_prev[mask][i][eout]
        entry = ends[i][ein]; dir_flag = 0 if ein == 0 else 1
        sequence.append(("curvy", (i, dir_flag)))
        if pmask == 0 and pi == -1:
            sequence.append(("conn_from_A", (orig, entry)))
        else:
            prev_exit = ends[pi][peout]
            sequence.append(("conn", (prev_exit, entry)))
        mask, i, eout = pmask, pi, peout
    sequence.reverse()
    return best_tuple, sequence

# ========= 最短 “禁止エッジ付き” =========
def shortest_path_avoiding(G, s, t,
                           banned_undirected=frozenset(),
                           banned_nodes=frozenset(),
                           soft_nodes=None,
                           soft_penalty=300.0):
    soft_nodes = set(soft_nodes or [])
    def weight(u, v, d):
        if v in banned_nodes:
            return float("inf")
        a, b = (u, v) if u < v else (v, u)
        if (a, b) in banned_undirected:
            return float("inf")
        base = float(d.get("cost_len", d.get("length", float("inf"))))
        if v in soft_nodes:
            base += float(soft_penalty)
        return base
    try:
        length, path = nx.single_source_dijkstra(G, s, target=t, weight=weight)
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return None
    if not path:
        return None
    turns = count_turns_on_path(G, path, TURN_THRESH_DEG)
    pure_len = sum_original_length_on_path(G, path)
    return {"path": path,
            "cost": float(sum_cost_len_on_path(G, path)),
            "turns": int(turns),
            "dist": float(pure_len)}

def first_undirected_edge(nodes):
    if len(nodes) < 2: return None
    u, v = nodes[0], nodes[1]
    return (u, v) if u < v else (v, u)

def last_undirected_edge(nodes):
    if len(nodes) < 2: return None
    u, v = nodes[-2], nodes[-1]
    return (u, v) if u < v else (v, u)

def prefix_undirected_edges(nodes, k):
    out = set()
    for u, v in zip(nodes[:-1], nodes[1:]):
        if k <= 0: break
        a, b = (u, v) if u < v else (v, u)
        out.add((a, b)); k -= 1
    return out

def adjacent_curvy_indices_in_sequence(sequence):
    adj, prev_item = {}, None
    for item in sequence:
        kind, info = item
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info; adj.setdefault((a, b), set())
        if kind == "curvy":
            i, _ = info
            if prev_item is not None and prev_item[0] in ("conn_from_A", "conn"):
                a, b = prev_item[1]
                adj.setdefault((a, b), set()).add(i)
        prev_item = item
    return adj

# ========= 実ノード列に展開（クリップ対応） =========
def build_full_node_path(sequence, routes_sel, path_lookup, curvy_clip=None):
    curvy_clip = curvy_clip or {}
    nodes, used_route_ids = [], []
    for kind, info in sequence:
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info; seg = path_lookup.get((a, b))
            if seg is None:
                dprint(f"[assemble] MISSING connector {a}->{b}")
                return None, None
            nodes += seg if not nodes else (seg[1:] if nodes[-1] == seg[0] else seg)
        elif kind == "curvy":
            i, dir_flag = info; r = routes_sel[i]
            seg = curvy_nodes_dir_clipped(r, dir_flag, curvy_clip.get(i))
            if not seg or len(seg) < 2:
                dprint(f"[assemble] TOO-SHORT curvy#{r['route_id']} after clip (len={0 if not seg else len(seg)})")
                return None, None
            nodes += seg if not nodes else (seg[1:] if nodes[-1] == seg[0] else seg)
            used_route_ids.append(r["route_id"])
    return nodes, used_route_ids

# ========= 可視化 =========
def plot_full(G_multi, full_nodes, routes_used_ids, curvy_routes, A_latlon, B_latlon):
    fig, ax = ox.plot_graph(G_multi, node_size=0, edge_color="#E0E0E0",
                            edge_linewidth=0.6, bgcolor="white", show=False, close=False)
    # すべての curvy を薄紫
    for r in curvy_routes:
        geom = r["geometry"]
        if hasattr(geom, "geoms"):
            for g in geom.geoms:
                x, y = g.xy; ax.plot(x, y, lw=1.2, color="#9B59B6", alpha=0.35)
        else:
            x, y = geom.xy; ax.plot(x, y, lw=1.2, color="#9B59B6", alpha=0.35)
        p = geom.representative_point()
        ax.text(p.x, p.y, f"#{r['route_id']} ({r.get('highway_top') or 'n/a'})", fontsize=8,
                color="#2C3E50", ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7))
    xs, ys = [], []
    for u, v in zip(full_nodes[:-1], full_nodes[1:]):
        data_dict = G_multi.get_edge_data(u, v, default=None)
        if data_dict:
            ed = min(data_dict.values(), key=lambda d: d.get("length", 1e9))
            geom = ed.get("geometry")
            if isinstance(geom, LineString):
                x, y = geom.xy; xs += list(x); ys += list(y); ax.plot(x, y, lw=3.0, color="#2C3E50", alpha=0.95); continue
        x0, y0 = G_multi.nodes[u]["x"], G_multi.nodes[u]["y"]
        x1, y1 = G_multi.nodes[v]["x"], G_multi.nodes[v]["y"]
        xs += [x0, x1]; ys += [y0, y1]
        ax.plot([x0, x1], [y0, y1], lw=3.0, color="#2C3E50", alpha=0.95)
    ax.scatter([A_latlon[1]], [A_latlon[0]], s=40, c="black", marker="o", zorder=5)
    ax.scatter([B_latlon[1]], [B_latlon[0]], s=40, c="red", marker="x", zorder=5)
    if ZOOM_TO_ROUTE and xs and ys:
        pad_x = (max(xs) - min(xs)) * 0.10; pad_y = (max(ys) - min(ys)) * 0.10
        ax.set_xlim(min(xs) - pad_x, max(xs) + pad_x); ax.set_ylim(min(ys) - pad_y, max(ys) + pad_y)
    plt.title("A→B with AB-ellipse + Min-Turns Lexicographic + No-Repeat (edge-avoid)")
    plt.tight_layout(); plt.show()

def make_interactive_map(G_multi, full_nodes, curvy_routes, used_route_ids, origin_latlon, dest_latlon, html_out="ab_curvy_route_map.html"):
    import folium
    from folium import Map, Marker, PolyLine
    from folium.plugins import MousePosition, MiniMap, Fullscreen

    m = Map(location=[origin_latlon[0], origin_latlon[1]], zoom_start=11, tiles="OpenStreetMap", control_scale=True)

    all_curvy_layer   = folium.FeatureGroup(name="Curvy (candidates)", show=True)
    used_curvy_layer  = folium.FeatureGroup(name="Curvy (used)", show=True)
    route_layer       = folium.FeatureGroup(name="A→B Route", show=True)
    bounds = []

    def add_bounds_from_xy(xs, ys):
        for x, y in zip(xs, ys): bounds.append((y, x))

    for r in curvy_routes:
        geom = r["geometry"]
        hw_top = r.get("highway_top") or "n/a"
        mix = r.get("highway_counts") or {}
        mix_str = ", ".join(f"{k}:{v}" for k, v in list(mix.items())[:3])
        tooltip = f"curvy #{r['route_id']} len={r['length_m']:.0f}m curv={r['curv_mean']:.3f} hw={hw_top}" + (f" mix=[{mix_str}]" if mix_str else "")
        def draw_lines(g, layer, weight, color, opacity):
            x, y = g.xy; latlons = [(yy, xx) for xx, yy in zip(x, y)]
            PolyLine(latlons, weight=weight, color=color, opacity=opacity, tooltip=tooltip).add_to(layer); add_bounds_from_xy(x, y)
        if hasattr(geom, "geoms"):
            for g in geom.geoms: draw_lines(g, all_curvy_layer, 3, "#b28dd6", 0.35)
        else:
            draw_lines(geom, all_curvy_layer, 3, "#b28dd6", 0.35)

    used_set = set(used_route_ids)
    for r in curvy_routes:
        if r["route_id"] not in used_set: continue
        geom = r["geometry"]; tooltip = f"USED curvy #{r['route_id']} len={r['length_m']:.0f}m"
        def draw_lines(g, layer, weight, color, opacity):
            x, y = g.xy; latlons = [(yy, xx) for xx, yy in zip(x, y)]
            PolyLine(latlons, weight=6, color=color, opacity=opacity, tooltip=tooltip).add_to(layer)
        if hasattr(geom, "geoms"):
            for g in geom.geoms: draw_lines(g, used_curvy_layer, 6, "#7c3aed", 0.95)
        else:
            draw_lines(geom, used_curvy_layer, 6, "#7c3aed", 0.95)

    route_latlons = [(G_multi.nodes[n]["y"], G_multi.nodes[n]["x"]) for n in full_nodes]
    PolyLine(route_latlons, weight=7, color="#1f2937", opacity=0.95, tooltip="A→B route").add_to(route_layer)
    for lat, lon in route_latlons: bounds.append((lat, lon))

    Marker(origin_latlon, tooltip="Start (A)", icon=folium.Icon(color="black")).add_to(route_layer)
    Marker(dest_latlon,   tooltip="Goal (B)",  icon=folium.Icon(color="red")).add_to(route_layer)
    all_curvy_layer.add_to(m); used_curvy_layer.add_to(m); route_layer.add_to(m)
    folium.LayerControl(collapsed=False).add_to(m); MiniMap(toggle_display=True).add_to(m); Fullscreen().add_to(m)
    MousePosition(lat_formatter="function(num){return L.Util.formatNum(num, 6);}",
                  lng_formatter="function(num){return L.Util.formatNum(num, 6);}").add_to(m)
    if bounds: m.fit_bounds(bounds, padding=(20, 20))

    m.save(html_out)
    globals()["LAST_FOLIUM_MAP"] = m
    return m, html_out

# ---- ここから core に追加（または置換） ----
def discover_curvy_candidates(
    origin_latlon,
    dest_latlon,
    require_n=5,
    min_length_m=500.0,
    save_preview_html="curvy_candidates_preview.html",
):
    global ORIGIN_LATLON, DEST_LATLON, MIN_LENGTH_M, REQUIRE_N, CENTER_LATLON, DISCOVERY_CTX

    ORIGIN_LATLON = origin_latlon
    DEST_LATLON   = dest_latlon
    MIN_LENGTH_M  = float(min_length_m)
    REQUIRE_N     = int(require_n)
    CENTER_LATLON = origin_latlon

    print("Loading base graph for candidate discovery...")
    G_multi = ox.graph_from_point(CENTER_LATLON, dist=SEARCH_RADIUS_M,
                                  network_type=NETWORK_TYPE, simplify=True)

    from osmnx import convert
    G = convert.to_digraph(G_multi)
    for u, v, d in G.edges(data=True):
        if "length" not in d:
            x0, y0 = G.nodes[u]["x"], G.nodes[u]["y"]; x1, y1 = G.nodes[v]["x"], G.nodes[v]["y"]
            d["length"] = LineString([(x0, y0), (x1, y1)]).length
        if CONNECTOR_FORBID_HIGHWAYS and edge_has_highway_in(d, EXCLUDE_HIGHWAY_CLASSES):
            d["cost_len"] = 1e12
        else:
            pen = highway_penalty_for_edge(d, CONNECTOR_HIGHWAY_PENALTY_M)
            d["cost_len"] = float(d["length"]) + pen

    orig = ox.distance.nearest_nodes(G, X=ORIGIN_LATLON[1], Y=ORIGIN_LATLON[0])
    dest = ox.distance.nearest_nodes(G, X=DEST_LATLON[1],   Y=DEST_LATLON[0])

    print("Collecting curvy edges pool...")
    segs = collect_curvy_edges(
        G_multi,
        min_length_m=MIN_LENGTH_M,
        exclude_short_intersection_links=EXCLUDE_SHORT_INTERSECTION_LINKS,
        inter_link_max_len_m=INTERSECTION_LINK_MAX_LEN_M,
        drop_narrow=True,
    )
    if not segs:
        print("No edges passed base filters. Relax thresholds.")
        DISCOVERY_CTX = {"G_multi": G_multi, "G": G, "orig": orig, "dest": dest,
                         "candidates": [], "origin_latlon": ORIGIN_LATLON, "dest_latlon": DEST_LATLON}
        return [], None

    print("Searching with AB-ellipse gate and percentile fallback...")
    p = P_INIT
    routes_hit = []
    while p >= P_MIN:
        picks_p, thr_p = pick_curvy(segs, percentile=p)
        H_p = build_curvy_subgraph(G_multi, picks_p)
        routes_p = decompose_into_routes(H_p)
        if EXCLUDE_DEADEND_CURVY:
            routes_p = [r for r in routes_p if both_endpoints_roundabout_safe(G_multi, r)]
        routes_p = filter_curvy_by_AB_ellipse(routes_p, ORIGIN_LATLON, DEST_LATLON)
        print(f"  p={p}: candidates in ellipse = {len(routes_p)} (thr={thr_p:.4f})")
        if len(routes_p) >= REQUIRE_N:
            routes_hit = routes_p
            break
        p -= P_STEP

    if not routes_hit:
        print(f"Not enough curvy inside ellipse down to p={P_MIN}.")
        DISCOVERY_CTX = {"G_multi": G_multi, "G": G, "orig": orig, "dest": dest,
                         "candidates": [], "origin_latlon": ORIGIN_LATLON, "dest_latlon": DEST_LATLON}
        return [], None

    import folium
    from folium import Map, Marker, PolyLine
    from folium.plugins import MousePosition, MiniMap, Fullscreen

    mid_lat = (ORIGIN_LATLON[0] + DEST_LATLON[0]) / 2.0
    mid_lon = (ORIGIN_LATLON[1] + DEST_LATLON[1]) / 2.0
    m = Map(location=[mid_lat, mid_lon], zoom_start=12, tiles="OpenStreetMap", control_scale=True)

    layer_curvy = folium.FeatureGroup(name="Curvy candidates", show=True)
    bounds = []

    def add_bounds_xy(xs, ys):
        for x, y in zip(xs, ys):
            bounds.append((y, x))

    for r in routes_hit:
        geom = r["geometry"]
        tooltip = f"curvy #{r['route_id']} len={r['length_m']:.0f}m curv={r['curv_mean']:.3f} hw={r.get('highway_top') or 'n/a'}"
        def draw_one(g):
            x, y = g.xy
            latlons = [(yy, xx) for xx, yy in zip(x, y)]
            PolyLine(latlons, weight=3, color="#b28dd6", opacity=0.55, tooltip=tooltip).add_to(layer_curvy)
            add_bounds_xy(x, y)
        if hasattr(geom, "geoms"):
            for g in geom.geoms: draw_one(g)
        else:
            draw_one(geom)

    Marker(ORIGIN_LATLON, tooltip="Start (A)", icon=folium.Icon(color="black", icon="play")).add_to(m)
    Marker(DEST_LATLON,   tooltip="Goal (B)",  icon=folium.Icon(color="red",   icon="flag")).add_to(m)
    bounds.extend([ORIGIN_LATLON, DEST_LATLON])

    layer_curvy.add_to(m)
    folium.LayerControl(collapsed=False).add_to(m)
    MiniMap(toggle_display=True).add_to(m)
    Fullscreen().add_to(m)
    MousePosition(
        lat_formatter="function(num){return L.Util.formatNum(num, 6);}",
        lng_formatter="function(num){return L.Util.formatNum(num, 6);}"
    ).add_to(m)

    if bounds: m.fit_bounds(bounds, padding=(20, 20))

    html_path = None
    if save_preview_html:
        m.save(save_preview_html)
        html_path = save_preview_html

    globals()["LAST_FOLIUM_MAP"] = m

    DISCOVERY_CTX = {
        "G_multi": G_multi,
        "G": G,
        "orig": orig,
        "dest": dest,
        "candidates": routes_hit,
        "origin_latlon": ORIGIN_LATLON,
        "dest_latlon": DEST_LATLON,
    }

    return routes_hit, html_path

def _ensure_ctx_for_AB(origin_latlon, dest_latlon, require_n=None, min_length_m=None, force_refresh=False):
    """
    現在の DISCOVERY_CTX が A/B と不一致なら discover_curvy_candidates() を実行して更新する。
    戻り値: 最新の ctx（= DISCOVERY_CTX）
    """
    global DISCOVERY_CTX, ORIGIN_LATLON, DEST_LATLON, REQUIRE_N, MIN_LENGTH_M, CENTER_LATLON

    # 既存グローバルのデフォルトを補完
    _origin = tuple(origin_latlon) if origin_latlon is not None else ORIGIN_LATLON
    _dest   = tuple(dest_latlon)   if dest_latlon   is not None else DEST_LATLON
    _reqN   = int(require_n)       if require_n     is not None else REQUIRE_N
    _minL   = float(min_length_m)  if min_length_m  is not None else MIN_LENGTH_M

    need_refresh = force_refresh or (DISCOVERY_CTX is None)
    if not need_refresh:
        ctx = DISCOVERY_CTX
        if (ctx.get("origin_latlon") != _origin) or (ctx.get("dest_latlon") != _dest):
            need_refresh = True

    if need_refresh:
        # 他ルーチンとの整合のためグローバルも同期
        ORIGIN_LATLON = _origin
        DEST_LATLON   = _dest
        REQUIRE_N     = _reqN
        MIN_LENGTH_M  = _minL
        CENTER_LATLON = _origin

        routes_hit, _ = discover_curvy_candidates(
            origin_latlon=_origin,
            dest_latlon=_dest,
            require_n=_reqN,
            min_length_m=_minL,
            save_preview_html=None,   # UI用のプレビューは不要
        )
        if not routes_hit:
            print("No curvy candidates under new A/B. Try relaxing thresholds.")
            return None

    return DISCOVERY_CTX

def route_with_selection(
    selected_ids,
    fill_to_n=True,
    # ★ 追加引数：A/B と閾値（UIから渡せるように）
    origin_latlon=None,
    dest_latlon=None,
    require_n=None,
    min_length_m=None,
    force_refresh=False,
):
    """
    discover_curvy_candidates() の結果（DISCOVERY_CTX）を使い、
    selected_ids の curvy を優先して最適化→Folium保存まで行う。
    A/Bや閾値が指定され、現在の ctx と不一致なら自動で再検出する。
    """

    # ★★★ ここだけ新規：A/B に合わせて ctx を確実に最新化 ★★★
    ctx = _ensure_ctx_for_AB(
        origin_latlon=origin_latlon,
        dest_latlon=dest_latlon,
        require_n=require_n,
        min_length_m=min_length_m,
        force_refresh=force_refresh,
    )
    if not ctx or not isinstance(ctx.get("candidates"), list):
        print("No discovery context. Call discover_curvy_candidates() first.")
        return
    
    G_multi = ctx["G_multi"]; G = ctx["G"]; orig = ctx["orig"]; dest = ctx["dest"]
    origin_latlon = ctx["origin_latlon"]; dest_latlon = ctx["dest_latlon"]
    cand = ctx["candidates"]

    id2r = {r["route_id"]: r for r in cand}
    routes_sel = [id2r[i] for i in selected_ids if i in id2r]
    if fill_to_n and len(routes_sel) < REQUIRE_N:
        remain = [r for r in sorted(cand, key=lambda r: r["length_m"], reverse=True)
                  if r["route_id"] not in selected_ids]
        routes_sel += remain[: max(0, REQUIRE_N - len(routes_sel))]

    if not routes_sel:
        print("No curvy selected and no fallback available.")
        return

    # 端点集合
    terminals = [orig, dest] + [n for r in routes_sel for n in (r["u"], r["v"])]
    terminals = list(dict.fromkeys(t for t in terminals if t in G))

    # 多対多最短の前計算
    if TURN_AWARE_CONNECTOR:
        dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_turnaware(
            G, terminals,
            turn_thresh=TURN_THRESH_DEG,
            uturn_forbid=UTURN_FORBID_DEG,
            W=TURN_LEXICOGRAPHIC_WEIGHT
        )
    else:
        dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_with_turns(G, terminals)

    # ====== ここから既存の最適化ループ ======
    MAX_ITERS = 60
    mutable_cost = dict(cost_lookup)
    iter_count = 0
    drops_done = 0
    last_forbidden = None

    # ★ クリップ情報（entry/exit）を保持
    curvy_clip = {}  # { i(index in routes_sel): (entry_node, exit_node) }

    def seg_nodes_of(item):
        kind, info = item
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info
            return path_lookup.get((a, b))
        elif kind == "curvy":
            i, dir_flag = info
            r = routes_sel[i]
            return curvy_nodes_dir_clipped(r, dir_flag, curvy_clip.get(i))
        return None

    def choose_curvy_orientation_and_snaps(G, last_seg_nodes, route, next_target_node, used_edges=None):
        used_edges = used_edges or set()
        best = (1e9, 0, None, None)
        prev_bear = last_edge_bearing_of_path(G, last_seg_nodes) if last_seg_nodes else None

        for dir_flag in (0, 1):
            nodes = curvy_nodes_in_direction(route, dir_flag)
            entry_cands = end_candidates(route, as_entry=(dir_flag == 0))
            exit_cands  = end_candidates(route, as_entry=(dir_flag == 1))

            def entry_bearing_from(entry):
                if entry not in nodes: return None
                idx = nodes.index(entry)
                return bearing_from_node_step(G, nodes, idx)

            def exit_bearing_to(exit_node):
                if exit_node not in nodes: return None
                idx = nodes.index(exit_node)
                if idx == 0: return None
                return _bearing_uv(G, nodes[idx-1], nodes[idx])

            for e_entry in entry_cands:
                b_in = entry_bearing_from(e_entry)
                ang_in = _angle_diff_deg(prev_bear, b_in) if (prev_bear is not None and b_in is not None) else 0.0

                for e_exit in exit_cands:
                    other_end = route["v"] if e_entry in (route["u"], route["nodes"][0]) else route["u"]
                    alt = shortest_path_avoiding(
                        G, e_exit, next_target_node,
                        banned_undirected=used_edges,
                        banned_nodes=set(),
                        soft_nodes={other_end},
                        soft_penalty=300.0
                    )
                    b_out = None
                    if alt and len(alt["path"]) >= 2:
                        b_out = _bearing_uv(G, alt["path"][0], alt["path"][1])

                    curvy_exit_bear = exit_bearing_to(e_exit)
                    ang_out = _angle_diff_deg(curvy_exit_bear, b_out) if (curvy_exit_bear is not None and b_out is not None) else 0.0

                    score = ang_in + ang_out
                    dprint(f"[choose] curvy#{route['route_id']} dir={'u->v' if dir_flag==0 else 'v->u'} "
                           f"entry={e_entry} exit={e_exit} ang_in={ang_in:.1f} ang_out={ang_out:.1f} score={score:.1f}")
                    if score < best[0]:
                        best = (score, dir_flag, e_entry, e_exit)

        _, best_dir, best_entry, best_exit = best
        dprint(f"[choose] BEST curvy#{route['route_id']} "
               f"dir={'u->v' if best_dir==0 else 'v->u'} entry={best_entry} exit={best_exit}")
        return best_dir, best_entry, best_exit

    while True:
        iter_count += 1
        if iter_count > MAX_ITERS:
            print("Reached max iterations; accept best feasible so far.")
            break

        # === ① 順序・向きの初期シーケンス決定（DP or A→B線形） ===
        if ORDER_MODE == "ab_linear":
            best_tuple = None
            sequence = build_sequence_ab_linear(
                G_multi, routes_sel, orig, dest, origin_latlon, dest_latlon
            )
        else:
            best_tuple, sequence = optimize_order_orientation(
                routes_sel, orig, dest, mutable_cost, turns_lookup
            )

        if sequence is None:
            dprint(f"[dp] infeasible with current forbids; last_forbidden={last_forbidden}")
            if ENABLE_DROP_CURVY_ON_FORCED_UTURN and drops_done < MAX_CURVY_DROPS and last_forbidden is not None:
                # 禁止解除して再試行
                mutable_cost[last_forbidden] = cost_lookup.get(last_forbidden)

                # === ② 再試行でも同じ分岐を適用（ここが2回目の差し込みポイント） ===
                if ORDER_MODE == "ab_linear":
                    best_tuple = None
                    sequence = build_sequence_ab_linear(
                        G_multi, routes_sel, orig, dest, origin_latlon, dest_latlon
                    )
                else:
                    best_tuple, sequence = optimize_order_orientation(
                        routes_sel, orig, dest, mutable_cost, turns_lookup
                    )

                if sequence is None:
                    print("No feasible solution after tentative un-forbid; abort.")
                    return
                # 近傍のcurvyを1本ドロップしてやり直し
                adj = adjacent_curvy_indices_in_sequence(sequence)
                neighbor_idxs = list(adj.get(last_forbidden, [])) or [0]
                victim_idx = neighbor_idxs[0]
                victim_route = routes_sel[victim_idx]
                dprint(f"[drop-neighbor] drop curvy#{victim_route['route_id']} adjacent to {last_forbidden} and retry")
                print(f"[drop] Removing curvy #{victim_route['route_id']} to avoid forced U-turns.")
                routes_sel.pop(victim_idx); drops_done += 1

                # 前計算の更新
                terminals = [orig, dest] + [n for r in routes_sel for n in (r["u"], r["v"])]
                terminals = list(dict.fromkeys(t for t in terminals if t in G))
                if TURN_AWARE_CONNECTOR:
                    dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_turnaware(
                        G, terminals,
                        turn_thresh=TURN_THRESH_DEG,
                        uturn_forbid=UTURN_FORBID_DEG,
                        W=TURN_LEXICOGRAPHIC_WEIGHT
                    )
                else:
                    dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_with_turns(G, terminals)
                mutable_cost = dict(cost_lookup); last_forbidden = None; iter_count = 0
                continue
            else:
                print("No feasible solution. Try relaxing filters or allow dropping more curvy.")
                return

        # === 以降：重複/UTURN/代替探索・クリップ最適化 ===
        used_edges = set()
        changed = False
        last_seg_nodes = None
        new_path_lookup = dict(path_lookup); new_dist_lookup = dict(dist_lookup); new_turns_lookup = dict(turns_lookup)

        def _ensure_conn_and_update(a, b, used_edges_local, strict=False):
            alt = shortest_path_avoiding(G, a, b, banned_undirected=used_edges_local)
            if strict and not (alt and alt["path"]):
                last_path = alt["path"] if alt else None
                for kban in (1, 2, 3):
                    banned2 = set(used_edges_local) | prefix_undirected_edges(last_path or [], kban)
                    alt2 = shortest_path_avoiding(G, a, b, banned_undirected=banned2)
                    if alt2 and alt2["path"]:
                        alt = alt2
                        break
            if alt and alt["path"]:
                new_path_lookup[(a, b)]  = alt["path"]
                new_dist_lookup[(a, b)]  = alt["dist"]
                new_turns_lookup[(a, b)] = alt["turns"]
                mutable_cost[(a, b)]     = alt["cost"]
                return True
            return False

        for idx, item in enumerate(sequence):
            kind, info = item

            if kind == "curvy":
                i, dir_flag = info
                r = routes_sel[i]

                # 次ターゲット（後続コネクタの入口）
                if idx+1 < len(sequence) and sequence[idx+1][0] in ("conn", "conn_to_B"):
                    _, nxt = sequence[idx+1]
                    next_target = nxt[1]
                else:
                    next_target = dest

                # 入口/出口の最適化（クリップ確定）
                best_dir, best_entry, best_exit = choose_curvy_orientation_and_snaps(
                    G, last_seg_nodes, r, next_target, used_edges
                )
                prev_len = len(r["nodes"])
                curvy_clip[i] = (best_entry, best_exit)
                seg = curvy_nodes_dir_clipped(r, best_dir, curvy_clip.get(i))
                dprint(f"[clip] curvy#{r['route_id']} dir={'u->v' if best_dir==0 else 'v->u'} "
                       f"entry={best_entry} exit={best_exit} len_before={prev_len} len_after={len(seg)}")

                # 向き置換
                if best_dir != dir_flag:
                    dprint(f"[flip] curvy#{r['route_id']} dir change {('u->v' if dir_flag==0 else 'v->u')} "
                           f"-> {('u->v' if best_dir==0 else 'v->u')}")
                    dir_flag = best_dir
                    sequence[idx] = ("curvy", (i, dir_flag))

                # 前後コネクタの端点更新
                if idx-1 >= 0 and sequence[idx-1][0] in ("conn_from_A", "conn"):
                    a_prev, b_prev = sequence[idx-1][1]
                    if b_prev != best_entry:
                        dprint(f"[conn-fix] prev-conn {a_prev}->{b_prev} -> {a_prev}->{best_entry}")
                        replace_tuple_in_sequence(sequence, idx-1, (a_prev, best_entry))
                        _ensure_conn_and_update(a_prev, best_entry, used_edges_local=used_edges, strict=False)

                if idx+1 < len(sequence) and sequence[idx+1][0] in ("conn", "conn_to_B"):
                    a_next, b_next = sequence[idx+1][1]
                    if a_next != best_exit:
                        dprint(f"[conn-fix] next-conn {a_next}->{b_next} -> {best_exit}->{b_next}")
                        replace_tuple_in_sequence(sequence, idx+1, (best_exit, b_next))
                        _ensure_conn_and_update(best_exit, b_next, used_edges_local=used_edges, strict=False)

                # Uターン最終チェック
                if last_seg_nodes and is_uturn_between_segments(G, last_seg_nodes, seg, thresh_deg=UTURN_FORBID_DEG):
                    info_pair = _bearing_pair_info(G, last_seg_nodes, seg)
                    if info_pair:
                        b1, b2, ang = info_pair
                        dprint(f"[u-turn] curvy#{r['route_id']} entering causes U-turn "
                               f"(bearing prev={b1:.1f}°, next={b2:.1f}°, angle={ang:.1f}° >= {UTURN_FORBID_DEG}°)")
                    alt_seg = curvy_nodes_dir_clipped(r, 1 - dir_flag, curvy_clip.get(i))
                    if last_seg_nodes and not is_uturn_between_segments(G, last_seg_nodes, alt_seg, thresh_deg=UTURN_FORBID_DEG):
                        dprint(f"[u-turn-fix] try reversal works for curvy#{r['route_id']}")
                        sequence[idx] = ("curvy", (i, 1 - dir_flag))
                        new_dir = 1 - dir_flag
                        new_nodes = curvy_nodes_dir_clipped(r, new_dir, curvy_clip.get(i))
                        new_entry, new_exit = new_nodes[0], new_nodes[-1]

                        if idx-1 >= 0 and sequence[idx-1][0] in ("conn_from_A", "conn"):
                            kprev, (a_prev, b_prev) = sequence[idx-1]
                            if b_prev != new_entry:
                                replace_tuple_in_sequence(sequence, idx-1, (a_prev, new_entry))
                            _ensure_conn_and_update(a_prev, new_entry, used_edges_local=used_edges, strict=True)

                        if idx+1 < len(sequence) and sequence[idx+1][0] in ("conn", "conn_to_B"):
                            knext, (a_next, b_next) = sequence[idx+1]
                            if a_next != new_exit:
                                replace_tuple_in_sequence(sequence, idx+1, (new_exit, b_next))
                            _ensure_conn_and_update(new_exit, b_next, used_edges_local=used_edges, strict=True)
                        changed = True
                        break
                    else:
                        dprint(f"[drop-candidate] curvy#{r['route_id']} still hard U-turn after reversal check")
                        if ENABLE_DROP_CURVY_ON_FORCED_UTURN and drops_done < MAX_CURVY_DROPS:
                            victim_route = routes_sel[i]
                            print(f"[drop] Removing curvy #{victim_route['route_id']} (hard U-turn at junction).")
                            routes_sel.pop(i); drops_done += 1

                            terminals = [orig, dest] + [n for r2 in routes_sel for n in (r2["u"], r2["v"])]
                            terminals = list(dict.fromkeys(t for t in terminals if t in G))
                            if TURN_AWARE_CONNECTOR:
                                dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_turnaware(
                                    G, terminals,
                                    turn_thresh=TURN_THRESH_DEG,
                                    uturn_forbid=UTURN_FORBID_DEG,
                                    W=TURN_LEXICOGRAPHIC_WEIGHT
                                )
                            else:
                                dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_with_turns(G, terminals)
                            mutable_cost = dict(cost_lookup); last_forbidden = None; iter_count = 0; changed = True; break

                last_seg_nodes = seg
                continue

            # --- connector ---
            a, b = info
            seg = path_lookup.get((a, b))
            if seg is None or len(seg) < 2:
                print("Missing connector path; abort.")
                return
            eset = edge_set_from_node_path(seg)
            have_overlap = bool(eset & used_edges)
            have_uturn   = last_seg_nodes and is_uturn_between_segments(G, last_seg_nodes, seg, thresh_deg=UTURN_FORBID_DEG)

            if have_overlap:
                dprint(f"[overlap] connector {a}->{b} has {len(eset & used_edges)} shared undirected edges: "
                       f"{_fmt_edges((eset & used_edges))}")
            if have_uturn:
                info_pair = _bearing_pair_info(G, last_seg_nodes, seg)
                if info_pair:
                    b1, b2, ang = info_pair
                    dprint(f"[u-turn] connector {a}->{b} makes U-turn (prev={b1:.1f}°, next={b2:.1f}°, ang={ang:.1f}°)")

            if not have_overlap and not have_uturn:
                used_edges |= eset
                last_seg_nodes = seg
                continue

            banned = set(used_edges)
            if last_seg_nodes:
                le = last_undirected_edge(last_seg_nodes)
                if le: banned.add(le)

            def find_alt_avoiding_uturn(max_prefix_ban=10):
                alt = shortest_path_avoiding(G, a, b, banned_undirected=banned)
                if alt and alt["path"] and (not last_seg_nodes or not is_uturn_between_segments(G, last_seg_nodes, alt["path"], thresh_deg=UTURN_FORBID_DEG)):
                    dprint(f"[alt] connector {a}->{b} found direct alt (no extra bans)")
                    return alt
                last_path = alt["path"] if alt else None
                for kban in range(1, max_prefix_ban + 1):
                    to_forbid = prefix_undirected_edges(last_path, kban) if last_path else set()
                    dprint(f"[alt] connector {a}->{b} try prefix-ban={kban} forbid={_fmt_edges(to_forbid)}")
                    banned2 = banned | to_forbid
                    alt2 = shortest_path_avoiding(G, a, b, banned_undirected=banned2)
                    if alt2 and alt2["path"] and (not last_seg_nodes or not is_uturn_between_segments(G, last_seg_nodes, alt2["path"], thresh_deg=UTURN_FORBID_DEG)):
                        dprint(f"[alt] connector {a}->{b} success with prefix-ban={kban}")
                        return alt2
                return None

            alt = find_alt_avoiding_uturn(max_prefix_ban=10)
            if alt is not None and alt["path"]:
                alt_nodes = alt["path"]
                new_path_lookup[(a, b)]  = alt_nodes
                new_dist_lookup[(a, b)]  = alt["dist"]
                new_turns_lookup[(a, b)] = alt["turns"]
                mutable_cost[(a, b)]     = alt["cost"]
                used_edges |= edge_set_from_node_path(alt_nodes)
                last_seg_nodes = alt_nodes
                changed = True
                break

            dprint(f"[forbid] connector {a}->{b} forbidden (no alt avoiding overlap/U-turn)")
            print(f"[fallback] No alt avoiding overlap/U-turn for {(a, b)}; forbidding and retrying...")
            mutable_cost[(a, b)] = None
            last_forbidden = (a, b)
            changed = True
            break

        if not changed:
            path_lookup = new_path_lookup; dist_lookup = new_dist_lookup; turns_lookup = new_turns_lookup
            break
        path_lookup = new_path_lookup; dist_lookup = new_dist_lookup; turns_lookup = new_turns_lookup

        # ---- ノード列の生成・救済・後処理 ----
        full_nodes, used_ids = build_full_node_path(sequence, routes_sel, path_lookup, curvy_clip)
        if full_nodes is None:
            print("Failed to assemble full path from sequence.")
            return

        MAX_SALVAGE = 3
        salv_cnt = 0
        while salv_cnt < MAX_SALVAGE:
            res = salvage_reroute_from_prefix_once(
                G, G_multi,
                orig_node=orig, dest_node=dest,
                routes_sel=routes_sel,
                sequence=sequence,
                path_lookup=path_lookup,
                passed_route_ids_prefix=[],
                curvy_clip=curvy_clip
            )
            if res is None:
                print("[salvage] failed to reroute from prefix; stop.")
                break
            new_full, new_used, did_fix = res
            if not did_fix:
                break
            print(f"[salvage] duplicate found → rerouted from prefix (#{salv_cnt+1})")
            full_nodes, used_ids = new_full, new_used
            salv_cnt += 1

        if ENABLE_LOOP_ERASE:
            before = len(full_nodes)
            full_nodes = (loop_erase_pingpong(full_nodes) if LOOP_ERASE_MODE=="pingpong"
                          else loop_erase_full(loop_erase_pingpong(full_nodes)))
            after = len(full_nodes)
            if after < before:
                print(f"[loop-erase] removed {before - after} intermediate nodes")

        if PRINT_ROUTE_NODE_IDS:
            print_route_nodes(G_multi, full_nodes)
        if SAVE_ROUTE_NODE_CSV:
            save_route_nodes_csv(G_multi, full_nodes, SAVE_ROUTE_NODE_CSV)
        if SAVE_ROUTE_NODE_GEOJSON:
            save_route_nodes_geojson(G_multi, full_nodes, SAVE_ROUTE_NODE_GEOJSON)

        total_len_after  = sum_original_length_on_path(G, full_nodes)
        total_turns_after= count_turns_on_path(G, full_nodes, TURN_THRESH_DEG)
        print(f"[after loop-erase] distance={total_len_after/1000:.2f} km, turns(≥{TURN_THRESH_DEG}°)={total_turns_after}")

    # ---- 最終合成（ループを抜けた時点の sequence/path_lookup で再構築）----
    if 'full_nodes' not in locals() or full_nodes is None:
        full_nodes, used_ids = build_full_node_path(sequence, routes_sel, path_lookup, curvy_clip)
        if full_nodes is None:
            print("Failed to assemble full path (final).")
            return
        if ENABLE_LOOP_ERASE:
            full_nodes = (loop_erase_pingpong(full_nodes) if LOOP_ERASE_MODE=="pingpong"
                          else loop_erase_full(loop_erase_pingpong(full_nodes)))

    # 統計出力
    total_conn = total_curvy = 0.0
    total_turns = 0
    for kind, info in sequence:
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info
            d = dist_lookup.get((a, b))
            if d is None:
                ax, ay = G.nodes[a]["x"], G.nodes[a]["y"]; bx, by = G.nodes[b]["x"], G.nodes[b]["y"]
                d = ((ax - bx)**2 + (ay - by)**2) ** 0.5
            total_conn += float(d)
            p = path_lookup.get((a, b))
            if p is not None:
                total_turns += count_turns_on_path(G, p, TURN_THRESH_DEG)
        elif kind == "curvy":
            i, _ = info
            total_curvy += float(routes_sel[i]["length_m"])
    total_len = total_conn + total_curvy

    print("\n=== RESULT ===")
    print(f"Total distance: {total_len/1000.0:.2f} km")
    print(f"Turns (connectors, ≥{TURN_THRESH_DEG}°): {total_turns}")
    order = []
    for kind, info in sequence:
        if kind == "curvy":
            i, dir_flag = info
            r = routes_sel[i]
            order.append((r["route_id"], "u->v" if dir_flag == 0 else "v->u"))
    print("Curvy route IDs (order & direction):", order)

    # 地図出力
    if SAVE_FOLIUM_HTML:
        m, html_path = make_interactive_map(
            G_multi=G_multi,
            full_nodes=full_nodes,
            curvy_routes=routes_sel,
            used_route_ids=[rid for rid, _ in order],
            origin_latlon=origin_latlon,
            dest_latlon=dest_latlon,
            html_out=SAVE_FOLIUM_HTML
        )
        print(f"Saved: {html_path}")
        LAST_FOLIUM_MAP = m

# ========= メイン =========
def main():
    """
    A→B ルーティングのフル実行（候補探索→最適化→可視化）。
    ORDER_MODE:
      - "dp"        : 既存の順序+向き DP
      - "ab_linear" : Aに近い端→Bに近い端の線形順でシーケンス構築
    """
    global LAST_FOLIUM_MAP
    LAST_FOLIUM_MAP = None

    ox.settings.log_console = False
    ox.settings.use_cache = True
    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED); np.random.seed(RANDOM_SEED)

    print("Loading base graph...")
    G_multi = ox.graph_from_point(CENTER_LATLON, dist=SEARCH_RADIUS_M, network_type=NETWORK_TYPE, simplify=True)

    from osmnx import convert
    G = convert.to_digraph(G_multi)
    for u, v, d in G.edges(data=True):
        if "length" not in d:
            x0, y0 = G.nodes[u]["x"], G.nodes[u]["y"]; x1, y1 = G.nodes[v]["x"], G.nodes[v]["y"]
            d["length"] = LineString([(x0, y0), (x1, y1)]).length
        if CONNECTOR_FORBID_HIGHWAYS and edge_has_highway_in(d, EXCLUDE_HIGHWAY_CLASSES):
            d["cost_len"] = 1e12
        else:
            pen = highway_penalty_for_edge(d, CONNECTOR_HIGHWAY_PENALTY_M)
            d["cost_len"] = float(d["length"]) + pen

    orig = ox.distance.nearest_nodes(G, X=ORIGIN_LATLON[1], Y=ORIGIN_LATLON[0])
    dest = ox.distance.nearest_nodes(G, X=DEST_LATLON[1],   Y=DEST_LATLON[0])

    print("Collecting curvy edges pool...")
    segs = collect_curvy_edges(
        G_multi,
        min_length_m=MIN_LENGTH_M,
        exclude_short_intersection_links=EXCLUDE_SHORT_INTERSECTION_LINKS,
        inter_link_max_len_m=INTERSECTION_LINK_MAX_LEN_M,
        drop_narrow=True
    )
    if not segs:
        print("No edges passed base filters. Relax thresholds.")
        return

    print("Searching with AB-ellipse gate and percentile fallback...")
    p = P_INIT; routes_hit = []
    while p >= P_MIN:
        picks_p, thr_p = pick_curvy(segs, percentile=p, absolute_threshold=None)
        H_p = build_curvy_subgraph(G_multi, picks_p)
        routes_p = decompose_into_routes(H_p)
        if EXCLUDE_DEADEND_CURVY:
            routes_p = [r for r in routes_p if both_endpoints_roundabout_safe(G_multi, r)]
        routes_p = filter_curvy_by_AB_ellipse(routes_p, ORIGIN_LATLON, DEST_LATLON)
        print(f"  p={p}: candidates in ellipse = {len(routes_p)} (thr={thr_p:.4f})")
        if len(routes_p) >= REQUIRE_N:
            routes_hit = routes_p; break
        p -= P_STEP
    if not routes_hit:
        print(f"Not enough curvy inside ellipse down to p={P_MIN}. Consider widening ellipse or lowering thresholds.")
        return

    # 上位から REQUIRE_N 本だけ選ぶ（長い順）
    routes_sel = sorted(routes_hit, key=lambda r: r["length_m"], reverse=True)[:REQUIRE_N]

    # 端点集合
    terminals = [orig, dest] + [n for r in routes_sel for n in (r["u"], r["v"])]
    terminals = list(dict.fromkeys(t for t in terminals if t in G))

    print("Precomputing multi-source shortest paths (+turn penalties)...")
    if TURN_AWARE_CONNECTOR:
        dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_turnaware(
            G, terminals,
            turn_thresh=TURN_THRESH_DEG,
            uturn_forbid=UTURN_FORBID_DEG,
            W=TURN_LEXICOGRAPHIC_WEIGHT
        )
    else:
        dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_with_turns(G, terminals)

    print("Optimizing order & orientation with edge-avoid & u-turn fixes...")
    MAX_ITERS = 60
    mutable_cost = dict(cost_lookup)
    iter_count = 0
    drops_done = 0
    last_forbidden = None

    # 入口/出口のクリップ情報
    curvy_clip = {}

    def seg_nodes_of(item):
        kind, info = item
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info; return path_lookup.get((a, b))
        elif kind == "curvy":
            i, dir_flag = info; r = routes_sel[i]
            return curvy_nodes_dir_clipped(r, dir_flag, curvy_clip.get(i))
        return None

    def choose_curvy_orientation_and_snaps(G, last_seg_nodes, route, next_target_node, used_edges=None):
        used_edges = used_edges or set()
        best = (1e9, 0, None, None)
        prev_bear = last_edge_bearing_of_path(G, last_seg_nodes) if last_seg_nodes else None
        for dir_flag in (0, 1):
            nodes = curvy_nodes_in_direction(route, dir_flag)
            entry_cands = end_candidates(route, as_entry=(dir_flag == 0))
            exit_cands  = end_candidates(route, as_entry=(dir_flag == 1))
            def entry_bearing_from(entry):
                if entry not in nodes: return None
                idx = nodes.index(entry); return bearing_from_node_step(G, nodes, idx)
            def exit_bearing_to(exit_node):
                if exit_node not in nodes: return None
                idx = nodes.index(exit_node)
                if idx == 0: return None
                return _bearing_uv(G, nodes[idx-1], nodes[idx])
            for e_entry in entry_cands:
                b_in = entry_bearing_from(e_entry)
                ang_in = _angle_diff_deg(prev_bear, b_in) if (prev_bear is not None and b_in is not None) else 0.0
                for e_exit in exit_cands:
                    other_end = route["v"] if e_entry in (route["u"], route["nodes"][0]) else route["u"]
                    alt = shortest_path_avoiding(
                        G, e_exit, dest if ORDER_MODE=="ab_linear" and not path_lookup else dest,
                        banned_undirected=used_edges,
                        banned_nodes=set(),
                        soft_nodes={other_end},
                        soft_penalty=300.0
                    )
                    b_out = None
                    if alt and len(alt["path"]) >= 2:
                        b_out = _bearing_uv(G, alt["path"][0], alt["path"][1])
                    curvy_exit_bear = exit_bearing_to(e_exit)
                    ang_out = _angle_diff_deg(curvy_exit_bear, b_out) if (curvy_exit_bear is not None and b_out is not None) else 0.0
                    score = ang_in + ang_out
                    dprint(f"[choose] curvy#{route['route_id']} dir={'u->v' if dir_flag==0 else 'v->u'} "
                           f"entry={e_entry} exit={e_exit} ang_in={ang_in:.1f} ang_out={ang_out:.1f} score={score:.1f}")
                    if score < best[0]:
                        best = (score, dir_flag, e_entry, e_exit)
        _, best_dir, best_entry, best_exit = best
        dprint(f"[choose] BEST curvy#{route['route_id']} "
               f"dir={'u->v' if best_dir==0 else 'v->u'} entry={best_entry} exit={best_exit}")
        return best_dir, best_entry, best_exit

    while True:
        iter_count += 1
        if iter_count > MAX_ITERS:
            print("Reached max iterations; accept best feasible so far.")
            break

        # === (1) 順序・向きシーケンス決定（DP or A→B線形） ===
        if ORDER_MODE == "ab_linear":
            best_tuple = None
            sequence = build_sequence_ab_linear(
                G_multi, routes_sel, orig, dest, ORIGIN_LATLON, DEST_LATLON
            )
        else:
            best_tuple, sequence = optimize_order_orientation(
                routes_sel, orig, dest, mutable_cost, turns_lookup
            )

        if sequence is None:
            dprint(f"[dp] infeasible with current forbids; last_forbidden={last_forbidden}")
            if ENABLE_DROP_CURVY_ON_FORCED_UTURN and drops_done < MAX_CURVY_DROPS and last_forbidden is not None:
                # 一旦 forbid 解除して再試行
                mutable_cost[last_forbidden] = cost_lookup.get(last_forbidden)

                # === (2) 再試行でも同じ分岐を適用 ===
                if ORDER_MODE == "ab_linear":
                    best_tuple = None
                    sequence = build_sequence_ab_linear(
                        G_multi, routes_sel, orig, dest, ORIGIN_LATLON, DEST_LATLON
                    )
                else:
                    best_tuple, sequence = optimize_order_orientation(
                        routes_sel, orig, dest, mutable_cost, turns_lookup
                    )

                if sequence is None:
                    print("No feasible solution after tentative un-forbid; abort.")
                    return

                # 近傍のcurvyを1本ドロップしてやり直し
                adj = adjacent_curvy_indices_in_sequence(sequence)
                neighbor_idxs = list(adj.get(last_forbidden, [])) or [0]
                victim_idx = neighbor_idxs[0]; victim_route = routes_sel[victim_idx]
                dprint(f"[drop-neighbor] drop curvy#{victim_route['route_id']} adjacent to {last_forbidden} and retry")
                print(f"[drop] Removing curvy #{victim_route['route_id']} to avoid forced U-turns.")
                routes_sel.pop(victim_idx); drops_done += 1

                # 前計算更新
                terminals = [orig, dest] + [n for r in routes_sel for n in (r["u"], r["v"])]
                terminals = list(dict.fromkeys(t for t in terminals if t in G))
                if TURN_AWARE_CONNECTOR:
                    dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_turnaware(
                        G, terminals,
                        turn_thresh=TURN_THRESH_DEG,
                        uturn_forbid=UTURN_FORBID_DEG,
                        W=TURN_LEXICOGRAPHIC_WEIGHT
                    )
                else:
                    dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_with_turns(G, terminals)
                mutable_cost = dict(cost_lookup); last_forbidden = None; iter_count = 0
                continue
            else:
                print("No feasible solution. Try relaxing filters or allow dropping more curvy.")
                return

        # === 以降：重複/UTURN/代替探索・クリップ最適化 ===
        used_edges = set()
        changed = False
        last_seg_nodes = None
        new_path_lookup = dict(path_lookup); new_dist_lookup = dict(dist_lookup); new_turns_lookup = dict(turns_lookup)

        def _ensure_conn_and_update(a, b, used_edges_local, strict=False):
            alt = shortest_path_avoiding(G, a, b, banned_undirected=used_edges_local)
            if strict and not (alt and alt["path"]):
                last_path = alt["path"] if alt else None
                for kban in (1, 2, 3):
                    banned2 = set(used_edges_local) | prefix_undirected_edges(last_path or [], kban)
                    alt2 = shortest_path_avoiding(G, a, b, banned_undirected=banned2)
                    if alt2 and alt2["path"]:
                        alt = alt2; break
            if alt and alt["path"]:
                new_path_lookup[(a, b)]  = alt["path"]
                new_dist_lookup[(a, b)]  = alt["dist"]
                new_turns_lookup[(a, b)] = alt["turns"]
                mutable_cost[(a, b)]     = alt["cost"]
                return True
            return False

        for idx, item in enumerate(sequence):
            kind, info = item

            if kind == "curvy":
                i, dir_flag = info; r = routes_sel[i]

                # 次の接続先
                if idx+1 < len(sequence) and sequence[idx+1][0] in ("conn", "conn_to_B"):
                    _, nxt = sequence[idx+1]; next_target = nxt[1]
                else:
                    next_target = dest

                # entry/exit の最適化とクリップ
                best_dir, best_entry, best_exit = choose_curvy_orientation_and_snaps(
                    G, last_seg_nodes, r, next_target, used_edges
                )
                prev_len = len(r["nodes"])
                curvy_clip[i] = (best_entry, best_exit)
                seg = curvy_nodes_dir_clipped(r, best_dir, curvy_clip.get(i))
                dprint(f"[clip] curvy#{r['route_id']} dir={'u->v' if best_dir==0 else 'v->u'} "
                       f"entry={best_entry} exit={best_exit} len_before={prev_len} len_after={len(seg)}")

                # 向きの差し替え
                if best_dir != dir_flag:
                    dprint(f"[flip] curvy#{r['route_id']} dir change {('u->v' if dir_flag==0 else 'v->u')} "
                           f"-> {('u->v' if best_dir==0 else 'v->u')}")
                    dir_flag = best_dir
                    sequence[idx] = ("curvy", (i, dir_flag))

                # 前後コネクタ端点の修正
                if idx-1 >= 0 and sequence[idx-1][0] in ("conn_from_A", "conn"):
                    a_prev, b_prev = sequence[idx-1][1]
                    if b_prev != best_entry:
                        dprint(f"[conn-fix] prev-conn {a_prev}->{b_prev} -> {a_prev}->{best_entry}")
                        replace_tuple_in_sequence(sequence, idx-1, (a_prev, best_entry))
                        _ensure_conn_and_update(a_prev, best_entry, used_edges_local=used_edges, strict=False)
                if idx+1 < len(sequence) and sequence[idx+1][0] in ("conn", "conn_to_B"):
                    a_next, b_next = sequence[idx+1][1]
                    if a_next != best_exit:
                        dprint(f"[conn-fix] next-conn {a_next}->{b_next} -> {best_exit}->{b_next}")
                        replace_tuple_in_sequence(sequence, idx+1, (best_exit, b_next))
                        _ensure_conn_and_update(best_exit, b_next, used_edges_local=used_edges, strict=False)

                # Uターン最終チェック
                if last_seg_nodes and is_uturn_between_segments(G, last_seg_nodes, seg, thresh_deg=UTURN_FORBID_DEG):
                    info_pair = _bearing_pair_info(G, last_seg_nodes, seg)
                    if info_pair:
                        b1, b2, ang = info_pair
                        dprint(f"[u-turn] curvy#{r['route_id']} entering causes U-turn "
                               f"(prev={b1:.1f}°, next={b2:.1f}°, ang={ang:.1f}° ≥ {UTURN_FORBID_DEG}°)")
                    alt_seg = curvy_nodes_dir_clipped(r, 1 - dir_flag, curvy_clip.get(i))
                    if last_seg_nodes and not is_uturn_between_segments(G, last_seg_nodes, alt_seg, thresh_deg=UTURN_FORBID_DEG):
                        # 逆向きにして解消
                        sequence[idx] = ("curvy", (i, 1 - dir_flag))
                        new_nodes = curvy_nodes_dir_clipped(r, 1 - dir_flag, curvy_clip.get(i))
                        new_entry, new_exit = new_nodes[0], new_nodes[-1]
                        if idx-1 >= 0 and sequence[idx-1][0] in ("conn_from_A", "conn"):
                            a_prev, b_prev = sequence[idx-1][1]
                            if b_prev != new_entry:
                                replace_tuple_in_sequence(sequence, idx-1, (a_prev, new_entry))
                            _ensure_conn_and_update(a_prev, new_entry, used_edges_local=used_edges, strict=True)
                        if idx+1 < len(sequence) and sequence[idx+1][0] in ("conn", "conn_to_B"):
                            a_next, b_next = sequence[idx+1][1]
                            if a_next != new_exit:
                                replace_tuple_in_sequence(sequence, idx+1, (new_exit, b_next))
                            _ensure_conn_and_update(new_exit, b_next, used_edges_local=used_edges, strict=True)
                        changed = True; break
                    else:
                        # それでもダメならドロップ
                        if ENABLE_DROP_CURVY_ON_FORCED_UTURN and drops_done < MAX_CURVY_DROPS:
                            victim_route = routes_sel[i]
                            print(f"[drop] Removing curvy #{victim_route['route_id']} (hard U-turn at junction).")
                            routes_sel.pop(i); drops_done += 1
                            terminals = [orig, dest] + [n for r2 in routes_sel for n in (r2["u"], r2["v"])]
                            terminals = list(dict.fromkeys(t for t in terminals if t in G))
                            if TURN_AWARE_CONNECTOR:
                                dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_turnaware(
                                    G, terminals, turn_thresh=TURN_THRESH_DEG,
                                    uturn_forbid=UTURN_FORBID_DEG, W=TURN_LEXICOGRAPHIC_WEIGHT
                                )
                            else:
                                dist_lookup, turns_lookup, cost_lookup, path_lookup = multi_shortest_between_with_turns(G, terminals)
                            mutable_cost = dict(cost_lookup); last_forbidden = None; iter_count = 0; changed = True; break
                last_seg_nodes = seg
                continue

            # --- connector ---
            a, b = info; seg = path_lookup.get((a, b))
            if seg is None or len(seg) < 2:
                print("Missing connector path; abort."); return
            eset = edge_set_from_node_path(seg)
            have_overlap = bool(eset & used_edges)
            have_uturn   = last_seg_nodes and is_uturn_between_segments(G, last_seg_nodes, seg, thresh_deg=UTURN_FORBID_DEG)
            if have_overlap:
                dprint(f"[overlap] connector {a}->{b} has {len(eset & used_edges)} shared undirected edges: {_fmt_edges((eset & used_edges))}")
            if have_uturn:
                info_pair = _bearing_pair_info(G, last_seg_nodes, seg)
                if info_pair:
                    b1, b2, ang = info_pair
                    dprint(f"[u-turn] connector {a}->{b} makes U-turn (prev={b1:.1f}°, next={b2:.1f}°, ang={ang:.1f}°)")

            if not have_overlap and not have_uturn:
                used_edges |= eset; last_seg_nodes = seg; continue

            banned = set(used_edges)
            if last_seg_nodes:
                le = last_undirected_edge(last_seg_nodes)
                if le: banned.add(le)

            def find_alt_avoiding_uturn(max_prefix_ban=10):
                alt = shortest_path_avoiding(G, a, b, banned_undirected=banned)
                if alt and alt["path"] and (not last_seg_nodes or not is_uturn_between_segments(G, last_seg_nodes, alt["path"], thresh_deg=UTURN_FORBID_DEG)):
                    dprint(f"[alt] connector {a}->{b} found direct alt (no extra bans)")
                    return alt
                last_path = alt["path"] if alt else None
                for kban in range(1, max_prefix_ban + 1):
                    to_forbid = prefix_undirected_edges(last_path, kban) if last_path else set()
                    dprint(f"[alt] connector {a}->{b} try prefix-ban={kban} forbid={_fmt_edges(to_forbid)}")
                    banned2 = banned | to_forbid
                    alt2 = shortest_path_avoiding(G, a, b, banned_undirected=banned2)
                    if alt2 and alt2["path"] and (not last_seg_nodes or not is_uturn_between_segments(G, last_seg_nodes, alt2["path"], thresh_deg=UTURN_FORBID_DEG)):
                        dprint(f"[alt] connector {a}->{b} success with prefix-ban={kban}")
                        return alt2
                return None

            alt = find_alt_avoiding_uturn(max_prefix_ban=10)
            if alt is not None and alt["path"]:
                alt_nodes = alt["path"]
                new_path_lookup[(a, b)]  = alt_nodes
                new_dist_lookup[(a, b)]  = alt["dist"]
                new_turns_lookup[(a, b)] = alt["turns"]
                mutable_cost[(a, b)]     = alt["cost"]
                used_edges |= edge_set_from_node_path(alt_nodes)
                last_seg_nodes = alt_nodes
                changed = True
                break

            dprint(f"[forbid] connector {a}->{b} forbidden (no alt avoiding overlap/U-turn)")
            print(f"[fallback] No alt avoiding overlap/U-turn for {(a, b)}; forbidding and retrying...")
            mutable_cost[(a, b)] = None
            last_forbidden = (a, b)
            changed = True
            break

        if not changed:
            path_lookup = new_path_lookup; dist_lookup = new_dist_lookup; turns_lookup = new_turns_lookup
            break
        path_lookup = new_path_lookup; dist_lookup = new_dist_lookup; turns_lookup = new_turns_lookup

        # ノード列の合成と救済・後処理
        full_nodes, used_ids = build_full_node_path(sequence, routes_sel, path_lookup, curvy_clip)
        if full_nodes is None:
            print("Failed to assemble full path from sequence."); return

        MAX_SALVAGE = 3
        salv_cnt = 0
        while salv_cnt < MAX_SALVAGE:
            res = salvage_reroute_from_prefix_once(
                G, G_multi, orig_node=orig, dest_node=dest,
                routes_sel=routes_sel, sequence=sequence, path_lookup=path_lookup,
                passed_route_ids_prefix=[], curvy_clip=curvy_clip
            )
            if res is None:
                print("[salvage] failed to reroute from prefix; stop."); break
            new_full, new_used, did_fix = res
            if not did_fix:
                break
            print(f"[salvage] duplicate found → rerouted from prefix (#{salv_cnt+1})")
            full_nodes, used_ids = new_full, new_used
            salv_cnt += 1

        if ENABLE_LOOP_ERASE:
            before = len(full_nodes)
            full_nodes = (loop_erase_pingpong(full_nodes) if LOOP_ERASE_MODE=="pingpong"
                          else loop_erase_full(loop_erase_pingpong(full_nodes)))
            after = len(full_nodes)
            if after < before:
                print(f"[loop-erase] removed {before - after} intermediate nodes")

        if PRINT_ROUTE_NODE_IDS:
            print_route_nodes(G_multi, full_nodes)
        if SAVE_ROUTE_NODE_CSV:
            save_route_nodes_csv(G_multi, full_nodes, SAVE_ROUTE_NODE_CSV)
        if SAVE_ROUTE_NODE_GEOJSON:
            save_route_nodes_geojson(G_multi, full_nodes, SAVE_ROUTE_NODE_GEOJSON)

        total_len_after  = sum_original_length_on_path(G, full_nodes)
        total_turns_after= count_turns_on_path(G, full_nodes, TURN_THRESH_DEG)
        print(f"[after loop-erase] distance={total_len_after/1000:.2f} km, turns(≥{TURN_THRESH_DEG}°)={total_turns_after}")

    # 最終合成（保険）
    if 'full_nodes' not in locals() or full_nodes is None:
        full_nodes, used_ids = build_full_node_path(sequence, routes_sel, path_lookup, curvy_clip)
        if full_nodes is None:
            print("Failed to assemble full path (final)."); return
        if ENABLE_LOOP_ERASE:
            full_nodes = (loop_erase_pingpong(full_nodes) if LOOP_ERASE_MODE=="pingpong"
                          else loop_erase_full(loop_erase_pingpong(full_nodes)))

    # 統計と可視化
    total_conn = total_curvy = 0.0; total_turns = 0
    for kind, info in sequence:
        if kind in ("conn_from_A", "conn", "conn_to_B"):
            a, b = info; d = dist_lookup.get((a, b))
            if d is None:
                ax, ay = G.nodes[a]["x"], G.nodes[a]["y"]; bx, by = G.nodes[b]["x"], G.nodes[b]["y"]
                d = ((ax - bx)**2 + (ay - by)**2) ** 0.5
            total_conn += float(d)
            p = path_lookup.get((a, b))
            if p is not None: total_turns += count_turns_on_path(G, p, TURN_THRESH_DEG)
        elif kind == "curvy":
            i, _ = info; total_curvy += float(routes_sel[i]["length_m"])
    total_len = total_conn + total_curvy

    print("\n=== RESULT ===")
    print(f"Total distance: {total_len/1000.0:.2f} km")
    print(f"Turns (connectors, ≥{TURN_THRESH_DEG}°): {total_turns}")
    order = []
    for kind, info in sequence:
        if kind == "curvy":
            i, dir_flag = info; r = routes_sel[i]
            order.append((r["route_id"], "u->v" if dir_flag == 0 else "v->u"))
    print("Curvy route IDs (order & direction):", order)

    if PLOT_MATPLOTLIB:
        print("Plotting (matplotlib)...")
        plot_full(G_multi, full_nodes, [rid for rid, _ in order], routes_sel, ORIGIN_LATLON, DEST_LATLON)

    if SAVE_FOLIUM_HTML:
        print("Building interactive map (folium)...")
        m, html_path = make_interactive_map(
            G_multi=G_multi,
            full_nodes=full_nodes,
            curvy_routes=routes_sel,
            used_route_ids=[rid for rid, _ in order],
            origin_latlon=ORIGIN_LATLON,
            dest_latlon=DEST_LATLON,
            html_out=SAVE_FOLIUM_HTML
        )
        print(f"Saved: {html_path}")
        try:
            display(m)
        except Exception:
            pass

if __name__ == "__main__":
    main()
