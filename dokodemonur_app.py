# app.py
# Streamlit UI with KODO-inspired styling + Nürburgring branding
import io
import os
import base64
import streamlit as st
from streamlit_folium import st_folium
import folium
from folium.plugins import MousePosition, MiniMap, Fullscreen, Geocoder
from contextlib import redirect_stdout

# コア：run_with_globals を持ち、LAST_FOLIUM_MAP を更新する修正版
import dokodemonur as core

# -------------------- ページ設定 --------------------
st.set_page_config(
    page_title="どこでもニュル | ニュルはみんなの心の中にある",
    layout="wide",
)

# -------------------- KODOスタイル（CSS） --------------------
SOUL_RED = "#b11226"     # Soul Red Crystal（近似）
JET_BLACK = "#0b0b0b"    # Jet Black
METALLIC = "#303030"
WHITE = "#f5f5f5"

st.markdown(
    f"""
    <style>
      /* 背景：深い黒にほのかなグラデ、余白を広く */
      .stApp {{
        background: radial-gradient(1200px 600px at 20% -10%, #151515 0%, {JET_BLACK} 60%) !important;
        color: {WHITE};
      }}
      /* サイドバー */
      section[data-testid="stSidebar"] > div {{
        background: linear-gradient(180deg, #121212 0%, #0f0f0f 100%) !important;
        border-right: 1px solid #1d1d1d;
      }}
      /* 見出しと本文のタイポ */
      html, body, [class*="css"] {{
        font-family: "Noto Sans JP", system-ui, -apple-system, Segoe UI, Roboto, "Hiragino Kaku Gothic ProN", "Meiryo", sans-serif;
      }}
      h1, h2, h3, .app-title {{
        font-family: "Montserrat", "Noto Sans JP", ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
        letter-spacing: 0.02em;
      }}
      /* ボタン：ソウルレッド、ホバーでやや暗く */
      .stButton > button {{
        background: {SOUL_RED};
        color: #ffffff;
        border: none;
        border-radius: 14px;
        padding: 0.6rem 1.1rem;
        box-shadow: 0 6px 18px rgba(177, 18, 38, 0.25);
      }}
      .stButton > button:hover {{
        background: #8f0f1f;
      }}
      .stButton > button:focus {{
        outline: 2px solid #ffffff22;
      }}
      /* インフォカードの色味を少しダークに */
      .stAlert > div {{
        background: #141414 !important;
        border: 1px solid #262626 !important;
        color: {WHITE} !important;
      }}
      /* 罫線・区切り */
      hr {{
        border: none; height: 1px; background: #262626; margin: 1rem 0;
      }}
      /* ダウンロードボタンなどのボックス */
      .result-card {{
        background: #121212;
        border: 1px solid #232323;
        padding: 1rem;
        border-radius: 16px;
      }}
      /* ヘッダーバー */
      .brand-bar {{
        display: flex; align-items: center; gap: 16px;
        border-bottom: 1px solid #1b1b1b; padding: 10px 12px 18px 12px;
        margin-bottom: 10px;
      }}
      .brand-left {{
        display: flex; align-items: center; gap: 12px;
      }}
      .brand-title {{
        font-weight: 700; font-size: 1.6rem; line-height: 1.2;
      }}
      .brand-sub {{
        color: #c7c7c7; font-size: .95rem; margin-top: 4px;
      }}
      .pill {{
        display: inline-block; padding: 3px 10px; border-radius: 999px;
        background: #1b1b1b; border: 1px solid #262626; font-size: .8rem; color: #cfcfcf;
      }}
    </style>
    <!-- Google Fonts（CDN） -->
    <link href="https://fonts.googleapis.com/css2?family=Montserrat:wght@600;700&family=Noto+Sans+JP:wght@400;600;700&display=swap" rel="stylesheet">
    """,
    unsafe_allow_html=True,
)

# -------------------- ブランディングヘッダー --------------------
def _img_html_from_bytes(data: bytes, mime: str = "image/png", size: int = 42) -> str:
    b64 = base64.b64encode(data).decode("ascii")
    return f'<img src="data:{mime};base64,{b64}" alt="logo" style="height:{size}px;width:auto;display:block;">'

from typing import Optional

def render_brand_header(logo_bytes: Optional[bytes]):
    if logo_bytes:
        logo_html = _img_html_from_bytes(logo_bytes, size=44)
    else:
        logo_html = f'<div style="height:44px;width:44px;border-radius:50%;background:{SOUL_RED};box-shadow:0 6px 18px rgba(177,18,38,.3) inset;"></div>'
    st.markdown(
        f"""
        <div class="brand-bar">
          <div class="brand-left">
            {logo_html}
            <div>
              <div class="brand-title">どこでもニュル</div>
              <div class="brand-sub">A→B ルーティング（Uターン不可＋右左折最小→距離最小・楕円ゲート）</div>
            </div>
          </div>
          <div style="margin-left:auto">
            <span class="pill">KODO-inspired UI</span>
          </div>
        </div>
        """,
        unsafe_allow_html=True,
    )


# -------------------- ロゴ指定（アップロード or URL） --------------------
#with st.sidebar:
#    st.markdown("### ブランディング")
#    logo_file = st.file_uploader("ニュルブルクリンクのロゴをアップロード（PNG/JPG/SVG）", type=["png", "jpg", "jpeg", "svg"])
#    logo_url = st.text_input("ロゴ画像URL（任意）")

#logo_bytes = None
#if logo_file is not None:
#    logo_bytes = logo_file.read()
#elif logo_url:
    # ネット取得は環境により不許可のことがあるため、ここでは簡易フォールバック（未取得時はNoneのまま）
#    try:
#        import requests
#        r = requests.get(logo_url, timeout=5)
#        if r.ok:
#            logo_bytes = r.content
#    except Exception:
#        pass

#render_brand_header(logo_bytes)

# -------------------- セッション状態 --------------------
DEFAULT_ORIGIN = (43.730221548916596, 142.38388084900924)  # lat, lon
DEFAULT_DEST   = (43.768802682478,     142.47555914795294)

if "start_latlon" not in st.session_state:
    st.session_state.start_latlon = DEFAULT_ORIGIN
if "goal_latlon" not in st.session_state:
    st.session_state.goal_latlon = DEFAULT_DEST
if "assign_target" not in st.session_state:
    st.session_state.assign_target = "Start"

# -------------------- サイドバー（パラメータ） --------------------
with st.sidebar:
    st.header("パラメータ")
    st.session_state.assign_target = st.radio(
        "地図クリックの反映先",
        ["Start", "Goal"],
        horizontal=True,
    )
    require_n = st.number_input("curvy 経由本数（REQUIRE_N）",
                                min_value=1, max_value=20, value=5, step=1)
    min_len_m = st.number_input("curvy とみなす最低エッジ長 [m]（MIN_LENGTH_M）",
                                min_value=10.0, max_value=5000.0, value=500.0, step=10.0)
    with st.expander("数値で微調整（任意）"):
        slat = st.number_input("Start lat", value=float(st.session_state.start_latlon[0]), format="%.12f")
        slon = st.number_input("Start lon", value=float(st.session_state.start_latlon[1]), format="%.12f")
        glat = st.number_input("Goal  lat", value=float(st.session_state.goal_latlon[0]),  format="%.12f")
        glon = st.number_input("Goal  lon", value=float(st.session_state.goal_latlon[1]),  format="%.12f")
        st.session_state.start_latlon = (slat, slon)
        st.session_state.goal_latlon  = (glat, glon)

    c1, c2 = st.columns(2)
    if c1.button("Start 初期化"):
        st.session_state.start_latlon = DEFAULT_ORIGIN
        try: st.rerun()
        except AttributeError: st.experimental_rerun()
    if c2.button("Goal 初期化"):
        st.session_state.goal_latlon = DEFAULT_DEST
        try: st.rerun()
        except AttributeError: st.experimental_rerun()

#    run = st.button("ルート計算を実行", type="primary")

# -------------------- 入力マップ（クリックで Start/Goal） --------------------
st.caption("クリックで Start / Goal を指定。左のラジオで対象を切替。検索バーから地名検索も可。")

center_lat = (st.session_state.start_latlon[0] + st.session_state.goal_latlon[0]) / 2
center_lon = (st.session_state.start_latlon[1] + st.session_state.goal_latlon[1]) / 2

# KODOトーンに合わせてダークタイル
base_map = folium.Map(location=[center_lat, center_lon], zoom_start=12, control_scale=True, tiles="CartoDB Dark_Matter")
MiniMap(toggle_display=True).add_to(base_map)
Fullscreen().add_to(base_map)
MousePosition(
    lat_formatter="function(num){return L.Util.formatNum(num, 6);}",
    lng_formatter="function(num){return L.Util.formatNum(num, 6);}"
).add_to(base_map)
try:
    Geocoder(collapsed=True, position="topleft").add_to(base_map)
except Exception:
    pass

# Start / Goal マーカー
folium.Marker(
    st.session_state.start_latlon, tooltip="Start (A)",
    icon=folium.Icon(color="black", icon="play")
).add_to(base_map)
folium.Marker(
    st.session_state.goal_latlon, tooltip="Goal (B)",
    icon=folium.Icon(color="red", icon="flag")
).add_to(base_map)

map_state = st_folium(base_map, height=520, width=None, key="picker_map")

# クリック反映（即時再描画）
if map_state and map_state.get("last_clicked"):
    lat = float(map_state["last_clicked"]["lat"])
    lon = float(map_state["last_clicked"]["lng"])
    if st.session_state.assign_target == "Start":
        st.session_state.start_latlon = (lat, lon)
    else:
        st.session_state.goal_latlon = (lat, lon)
    try: st.rerun()
    except AttributeError: st.experimental_rerun()

scol, gcol = st.columns(2)
scol.info(f"Start (A): lat={st.session_state.start_latlon[0]:.6f}, lon={st.session_state.start_latlon[1]:.6f}")
gcol.info(f"Goal  (B): lat={st.session_state.goal_latlon[0]:.6f}, lon={st.session_state.goal_latlon[1]:.6f}")
st.markdown("<hr/>", unsafe_allow_html=True)

# セッションキーを追加
if "curvy_candidates" not in st.session_state:
    st.session_state.curvy_candidates = []
if "selected_ids" not in st.session_state:
    st.session_state.selected_ids = []

st.subheader("ステップ1：curvy 候補を検出")
c1, c2 = st.columns([1,1])
if c1.button("候補を検出する", type="primary"):
    # ログを取りたい場合は redirect_stdout を使ってもOK
    cands, preview_html = core.discover_curvy_candidates(
        origin_latlon=st.session_state.start_latlon,
        dest_latlon=st.session_state.goal_latlon,
        require_n=int(require_n),
        min_length_m=float(min_len_m),
        save_preview_html="curvy_candidates_preview.html",
    )
    st.session_state.curvy_candidates = cands
    if preview_html and os.path.exists(preview_html):
        with open(preview_html, "r", encoding="utf-8") as f:
            st.components.v1.html(f.read(), height=520, scrolling=True)

# 候補があれば、マルチセレクトUI
if st.session_state.curvy_candidates:
    options = [
        f"#{r['route_id']} | {r['length_m']:.0f}m | curv={r['curv_mean']:.3f} | {r.get('highway_top') or 'n/a'}"
        for r in st.session_state.curvy_candidates
    ]
    id_lookup = { 
        f"#{r['route_id']} | {r['length_m']:.0f}m | curv={r['curv_mean']:.3f} | {r.get('highway_top') or 'n/a'}": r["route_id"]
        for r in st.session_state.curvy_candidates
    }
    selected_labels = st.multiselect("経由したい curvy を選んでください（複数可）", options, default=[])
    st.session_state.selected_ids = [id_lookup[s] for s in selected_labels]
else:
    st.info("先に『候補を検出する』を押してください。")

st.subheader("ステップ2：選んだ curvy を固定してルーティング")
fill_to_n = st.checkbox("必要本数 (REQUIRE_N) に満たない場合は長さ順で自動補完する", value=True)

if st.button("この選択でルート最適化", type="primary"):

    # 実行ログの表示先
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            core.route_with_selection(st.session_state.selected_ids, fill_to_n=fill_to_n)
    except Exception as e:
        st.error(f"実行中にエラー: {e}")
    finally:
        logs = buf.getvalue()
        if logs:
            st.code(logs, language="text")

    # 地図表示（HTML埋め込みでOK）
    html_name = core.SAVE_FOLIUM_HTML or "ab_curvy_route_map.html"
    if html_name and os.path.exists(html_name):
        with open(html_name, "r", encoding="utf-8") as f:
            st.components.v1.html(f.read(), height=620, scrolling=True)


# -------------------- 実行 & 結果表示 --------------------
log_area = st.empty()
result_area = st.container()

def run_and_render():
    html_name = core.SAVE_FOLIUM_HTML or "ab_curvy_route_map.html"
    csv_name  = core.SAVE_ROUTE_NODE_CSV or "route_nodes.csv"
    geo_name  = core.SAVE_ROUTE_NODE_GEOJSON or "route_nodes.geojson"

    # 既存出力の掃除
    for f in [html_name, csv_name, geo_name]:
        try:
            if f and os.path.exists(f): os.remove(f)
        except Exception:
            pass

    # 実行
    buf = io.StringIO()
    with st.spinner("経路を最適化中..."):
        try:
            with redirect_stdout(buf):
                core.run_with_globals(
                    origin_latlon=st.session_state.start_latlon,
                    dest_latlon=st.session_state.goal_latlon,
                    require_n=int(require_n),
                    min_length_m=float(min_len_m),
                )
        except Exception as e:
            logs = buf.getvalue()
            if logs: log_area.code(logs, language="text")
            st.error(f"実行中にエラー: {e}")
            return
        finally:
            logs = buf.getvalue()
            if logs: log_area.code(logs, language="text")

    # 地図描画（HTML埋め込み：最も安定）
    fmap = getattr(core, "LAST_FOLIUM_MAP", None)
    if fmap is not None and isinstance(fmap, folium.Map):
        try:
            html_str = fmap.get_root().render()
            st.markdown("### ルート地図（結果）")
            st.components.v1.html(html_str, height=620, scrolling=True)
        except Exception as e:
            st.warning(f"直接埋め込みに失敗: {e}. 保存HTMLにフォールバックします。")
            if html_name and os.path.exists(html_name):
                with open(html_name, "r", encoding="utf-8") as f:
                    html = f.read()
                st.components.v1.html(html, height=620, scrolling=True)
            else:
                st.error("地図HTMLが見つかりませんでした。")
    elif html_name and os.path.exists(html_name):
        with open(html_name, "r", encoding="utf-8") as f:
            html = f.read()
        st.markdown("### ルート地図（結果｜HTMLファイル埋め込み）")
        st.components.v1.html(html, height=620, scrolling=True)
    else:
        st.warning("地図を表示できませんでした。ログにエラーがないか確認してください。")

    # ダウンロード（KODO風カードに格納）
    with result_area:
        st.markdown('<div class="result-card">', unsafe_allow_html=True)
        cols = st.columns(3)
        if csv_name and os.path.exists(csv_name):
            with open(csv_name, "rb") as f:
                cols[0].download_button("ノード列CSVをダウンロード", data=f, file_name=csv_name, mime="text/csv")
        if geo_name and os.path.exists(geo_name):
            with open(geo_name, "rb") as f:
                cols[1].download_button("ノード列GeoJSONをダウンロード", data=f, file_name=geo_name, mime="application/geo+json")
        if html_name and os.path.exists(html_name):
            with open(html_name, "rb") as f:
                cols[2].download_button("地図HTMLをダウンロード", data=f, file_name=html_name, mime="text/html")
        st.markdown('</div>', unsafe_allow_html=True)

#if run:
#    run_and_render()
#else:
#    st.info("地図で Start / Goal を指定し、左の **[ルート計算を実行]** を押してください。")
st.info("1) 左で『候補を検出する』→ 2) 下でカーブを選択 → 3) 『この選択でルート最適化』の順に操作してください。")