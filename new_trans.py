import os
import geopandas as gpd
import numpy as np
from shapely.geometry import mapping, shape
from pyproj import Transformer
import json


# ----------------- 地理函数 -----------------
WGS84_A = 6378137.0
WGS84_F = 1/298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)

def geodetic_to_ecef(lat_deg, lon_deg, h=0.0):
    lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * (np.sin(lat)**2))
    X = (N + h) * np.cos(lat) * np.cos(lon)
    Y = (N + h) * np.cos(lat) * np.sin(lon)
    Z = (N * (1 - WGS84_E2) + h) * np.sin(lat)
    return np.vstack((X, Y, Z)).T

def ecef_to_enu_matrix(lat0_deg, lon0_deg):
    lat0 = np.deg2rad(lat0_deg); lon0 = np.deg2rad(lon0_deg)
    sl, cl = np.sin(lon0), np.cos(lon0)
    sp, cp = np.sin(lat0), np.cos(lat0)
    return np.array([
        [-sl,    cl,     0],
        [-sp*cl, -sp*sl, cp],
        [ cp*cl,  cp*sl, sp]
    ])

def estimate_similarity_transform(src_pts, dst_pts, with_scale=False):
    src, dst = np.asarray(src_pts), np.asarray(dst_pts)
    mu_src, mu_dst = src.mean(axis=0), dst.mean(axis=0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    H = src_c.T @ dst_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    s = 1.0
    if with_scale:
        s = S.sum() / (src_c**2).sum()
    t = mu_dst - s * (R @ mu_src)
    return s, R, t

def local_to_wgs84(local_xy, ctrl_local, ctrl_latlon, with_scale=False):
    ctrl_latlon = np.asarray(ctrl_latlon, dtype=float)
    lat0, lon0 = ctrl_latlon[0,1], ctrl_latlon[0,0]
    ecef_ctrl = geodetic_to_ecef(ctrl_latlon[:,1], ctrl_latlon[:,0], 0)
    ecef0 = geodetic_to_ecef(lat0, lon0, 0)[0]
    Renu = ecef_to_enu_matrix(lat0, lon0)
    enu_ctrl = (Renu @ (ecef_ctrl - ecef0).T).T[:, :2]

    s, R2, t = estimate_similarity_transform(ctrl_local, enu_ctrl, with_scale)
    mapped_enu = (s * (R2 @ local_xy.T)).T + t
    enu3 = np.hstack([mapped_enu, np.zeros((mapped_enu.shape[0],1))])
    ecef_mapped = (ecef0 + (Renu.T @ enu3.T).T)

    to_geod = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
    lon_lat = np.array([to_geod.transform(x,y,z) for x,y,z in ecef_mapped])
    return np.column_stack([lon_lat[:,1], lon_lat[:,0]])  # lat, lon

def transform_geometry(geom, func):
    """递归遍历 shapely geometry 并应用 func(x,y)"""
    g = mapping(geom)
    t = g["type"]

    if t == "Point":
        x, y = g["coordinates"][:2]  # Ensure only x, y are used
        lat, lon = func(x, y)
        return shape({"type": "Point", "coordinates": (lon, lat)})

    elif t == "LineString":
        coords = [func(pt[0], pt[1])[::-1] for pt in g["coordinates"]]
        return shape({"type": "LineString", "coordinates": coords})

    elif t == "Polygon":
        rings = []
        for ring in g["coordinates"]:
            coords = []
            for pt in ring:
                if isinstance(pt[0], (float, int)):
                    coords.append(func(pt[0], pt[1])[::-1])
                else:
                    for subpt in pt:
                        coords.append(func(subpt[0], subpt[1])[::-1])
            rings.append(coords)
        return shape({"type": "Polygon", "coordinates": rings})

    elif t in ("MultiPolygon", "MultiLineString", "GeometryCollection"):
        subgeoms = []
        for gg in g.get("geometries", g.get("coordinates", [])):
            subgeom = shape({"type": t.replace("Multi", ""), "coordinates": gg})
            subgeoms.append(transform_geometry(subgeom, func))
        return shape({
            "type": t,
            "geometries" if "geometries" in g else "coordinates":
                [mapping(sg)["coordinates"] for sg in subgeoms]
        })
    else:
        return geom
    
def get_ctro_coords(json_path):
    """
    从 JSON 文件中提取控制点坐标
    :param json_path: JSON 文件的路径
    :return: (ctrl_local, ctrl_latlon) 两个元组列表
    """
    # 1. 读取并解析 JSON 文件
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        print(f"读取文件出错: {e}")
        return [], []

    ctrl_local = []
    ctrl_latlon = []

    # 2. 遍历数据项
    for item in data:
        # 提取 CAD 坐标 (对应原 geom.x, geom.y)
        cad_xy = item.get("CAD_XY")
        if cad_xy and len(cad_xy) >= 2:
            # 存入 (x, y) 元组
            ctrl_local.append((float(cad_xy[0]), float(cad_xy[1])))

        # 提取地理坐标 (对应原 Longitude, Latitude)
        # JSON 格式为 [lon, lat, alt]，我们只取前两个
        lla = item.get("lon_lat_alt")
        if lla and len(lla) >= 2:
            # 存入 (longitude, latitude) 元组
            ctrl_latlon.append((float(lla[0]), float(lla[1])))

    return ctrl_local, ctrl_latlon
# ----------------- 主函数 -----------------
def process_shapefiles(input_dir, output_dir, coord_json, with_scale=False):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 1. 获取控制点
    ctrl_local, ctrl_latlon = get_ctro_coords(coord_json)
    if not ctrl_local or not ctrl_latlon:
        print("错误：无法加载控制点数据，请检查 JSON 文件。")
        return

    for root, _, files in os.walk(input_dir):
        for file in files:
            # --- 修改点 1: 检查后缀改为 .geojson ---
            if file.endswith(".geojson"): 
                input_geojson = os.path.join(root, file)
                # 输出文件名保持一致，或者你可以根据需要修改后缀
                output_file = os.path.join(output_dir, file)

                # --- 修改点 2: 读取 GeoJSON ---
                try:
                    gdf = gpd.read_file(input_geojson)
                    if gdf.empty:
                        print(f"Skipping empty file: {input_geojson}")
                        continue
                    print(f"Processing {input_geojson} with {len(gdf)} features")
                except Exception as e:
                    print(f"Error reading {input_geojson}: {e}")
                    continue

                # 定义转换闭包
                def xy_to_latlon(x, y):
                    res = local_to_wgs84(np.array([[x, y]]), ctrl_local, ctrl_latlon, with_scale)
                    return res[0]

                # 应用几何转换
                gdf["geometry"] = gdf["geometry"].apply(lambda g: transform_geometry(g, xy_to_latlon))

                # --- 修改点 3: 明确保存为 GeoJSON ---
                gdf.set_crs(epsg=4326, allow_override=True, inplace=True)
                gdf.to_file(output_file, driver="GeoJSON")
                print(f"Saved to {output_file}")

if __name__ == "__main__":
    input_directory = "trans"
    coord_json = "match.json"
    output_directory = "temp_gf"
    process_shapefiles(input_directory, output_directory, coord_json, with_scale=True)
