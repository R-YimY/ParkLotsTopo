"""
将 B3L/B3P 的 GeoJSON 经纬度坐标转为 ENU（东北天）坐标。
以 B3L 中经纬度最小值（min lon, min lat, 高程取0）为原点。
"""

import json
import numpy as np
import pymap3d as pm

# ── 辅助函数：从 GeoJSON 中提取所有 [lon, lat] ──────────────────────────
def extract_coords(geojson_path):
    """返回 (N, 2) 的 numpy 数组，每行 [lon, lat]"""
    with open(geojson_path) as f:
        data = json.load(f)

    pts = []
    for feat in data["features"]:
        geom = feat["geometry"]
        if geom["type"] == "Point":
            c = geom["coordinates"]
            pts.append([c[0], c[1]])
        elif geom["type"] in ("LineString", "MultiPoint"):
            for c in geom["coordinates"]:
                pts.append([c[0], c[1]])
        elif geom["type"] in ("Polygon", "MultiLineString"):
            for ring in geom["coordinates"]:
                for c in ring:
                    pts.append([c[0], c[1]])
        elif geom["type"] == "MultiPolygon":
            for poly in geom["coordinates"]:
                for ring in poly:
                    for c in ring:
                        pts.append([c[0], c[1]])
    return np.array(pts)


# ── 1. 读取 B3L，找经纬度最小值 ──────────────────────────────────────────
b3l_coords = extract_coords("B3L.geojson")
lon_min = b3l_coords[:, 0].min()
lat_min = b3l_coords[:, 1].min()
h_ref = 0.0  # 高程基准取 0

print(f"B3L 共 {len(b3l_coords)} 个点")
print(f"Lon 范围: [{b3l_coords[:,0].min():.6f}, {b3l_coords[:,0].max():.6f}]")
print(f"Lat 范围: [{b3l_coords[:,1].min():.6f}, {b3l_coords[:,1].max():.6f}]")
print(f"ENU 原点: lon={lon_min}, lat={lat_min}, h={h_ref}")


# ── 2. 转换函数 ──────────────────────────────────────────────────────────
def geojson_to_enu(geojson_path, ref_lon, ref_lat, ref_h=0.0):
    """
    读取 GeoJSON，将每个 feature 的坐标转为 ENU。
    保持原始 GeoJSON 结构，用 [E, N] 替换 [lon, lat]。
    返回新的 GeoJSON dict。
    """
    with open(geojson_path) as f:
        data = json.load(f)

    def convert_point(lon, lat, alt=None):
        if alt is None:
            alt = 0.0
        e, n, u = pm.geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_h)
        return [round(e, 6), round(n, 6)]

    for feat in data["features"]:
        geom = feat["geometry"]
        if geom["type"] == "Point":
            c = geom["coordinates"]
            geom["coordinates"] = convert_point(c[0], c[1], c[2] if len(c) > 2 else None)
        elif geom["type"] in ("LineString", "MultiPoint"):
            geom["coordinates"] = [
                convert_point(c[0], c[1], c[2] if len(c) > 2 else None)
                for c in geom["coordinates"]
            ]
        elif geom["type"] in ("Polygon", "MultiLineString"):
            geom["coordinates"] = [
                [convert_point(c[0], c[1], c[2] if len(c) > 2 else None) for c in ring]
                for ring in geom["coordinates"]
            ]
        elif geom["type"] == "MultiPolygon":
            geom["coordinates"] = [
                [
                    [convert_point(c[0], c[1], c[2] if len(c) > 2 else None) for c in ring]
                    for ring in poly
                ]
                for poly in geom["coordinates"]
            ]

    return data


# ── 3. 分别转换并保存 ────────────────────────────────────────────────────
for src in ("B3L", "B3P"):
    result = geojson_to_enu(f"{src}.geojson", lon_min, lat_min, h_ref)
    out_path = f"{src}_enu.geojson"
    with open(out_path, "w") as f:
        json.dump(result, f)
    # 统计
    all_e, all_n = [], []
    for feat in result["features"]:
        geom = feat["geometry"]
        coords = geom["coordinates"]
        if geom["type"] == "Point":
            all_e.append(coords[0]); all_n.append(coords[1])
        elif geom["type"] in ("LineString", "MultiPoint"):
            for c in coords: all_e.append(c[0]); all_n.append(c[1])
        elif geom["type"] in ("Polygon", "MultiLineString"):
            for ring in coords:
                for c in ring: all_e.append(c[0]); all_n.append(c[1])
        elif geom["type"] == "MultiPolygon":
            for poly in coords:
                for ring in poly:
                    for c in ring: all_e.append(c[0]); all_n.append(c[1])
    e_arr, n_arr = np.array(all_e), np.array(all_n)
    print(f"\n{src} → {out_path}")
    print(f"  East  范围: [{e_arr.min():.2f}, {e_arr.max():.2f}] m")
    print(f"  North 范围: [{n_arr.min():.2f}, {n_arr.max():.2f}] m")

print("\n✅ 完成，输出文件：B3L_enu.geojson / B3P_enu.geojson")
