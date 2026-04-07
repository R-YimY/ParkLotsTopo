from typing import List, Tuple, Dict, Set
from shapely import Point, LineString
import matplotlib.pyplot as plt
import json,os
import pymap3d
import numpy as np
from geometry import Point2D, Segment2D, Polyline2D

# ========== 最终封装类：仅保留两个核心对外接口 ==========
class IntersectionFinder:
    """
    【对外接口】
    1. run(): 执行计算流程
    2. get_intersection_info(): 获取交点信息（含坐标及连接的子多段线）
    3. get_split_polylines(): 获取拆分后的多段线数据
    4. visualize(): （可选）可视化调试
    """

    def __init__(self, input_polylines: List[List[Tuple[float, float]]]):
        """
        :param input_polylines: 输入格式：
            [
                [(x1, y1), (x2, y2), ...],  # 第0条多段线
                [(x1, y1), (x2, y2), ...],  # 第1条多段线
                ...
            ]
        """
        self._input_vertices = input_polylines
        self._original_polylines: List[Polyline2D] = []
        for idx, vertices in enumerate(input_polylines):
            if len(vertices) < 2:
                raise ValueError(f"多段线 {idx} 顶点数不足2")
            self._original_polylines.append(Polyline2D(vertices, poly_idx=idx))
        
        # 内部结果变量
        self._intersections: Dict[Point2D, List[Dict]] = {}
        self._split_polylines: List[Polyline2D] = []
        self._point_to_subpolys: Dict[Point2D, List[int]] = {}
        self._has_run = False


    # -------------------------------------------------------------------------
    # 内部私有方法（不对外）
    # -------------------------------------------------------------------------
    def _seg_intersect(self,seg_a_geom:LineString,seg_b_geom:LineString,dis_eps:float = 0.2):
        """判断两个线段之间的相交关系"""
        
        
        def extend_line(line: LineString, distance: float= 0.5) -> LineString:
            # 1. 提取坐标
            coords = list(line.coords)
            p_start = np.array(coords[0])
            p_end = np.array(coords[-1])
            
            # 2. 计算单位方向向量
            vec = p_end - p_start
            unit_vec = vec / np.linalg.norm(vec)
            
            # 3. 计算新端点
            new_start = p_start - unit_vec * distance
            new_end = p_end + unit_vec * distance
            
            return LineString([new_start, new_end])

        
        # 距离判断
        dis = seg_a_geom.distance(seg_b_geom)
        if dis > dis_eps:
            return None
        # 相交判断
        inter = seg_a_geom.intersection(seg_b_geom)
        if str(inter.geom_type).upper() != 'POINT':
            # 即距离接近，但是不相交,延长求交点
            seg_a_geom_extend = extend_line(seg_a_geom)
            seg_b_geom_extend = extend_line(seg_b_geom)
            new_inter = seg_a_geom_extend.intersection(seg_b_geom_extend)
            if str(new_inter.geom_type).upper() != 'POINT':
                return None
            result = new_inter

        else:
            result = inter
        
        return result


    def _find_all_intersections(self,eps = 1e-6) -> Dict[Point2D, List[Dict]]:
        intersections = {}
        polylines = self._original_polylines

        for poly_a_idx in range(len(polylines)):
            poly_a = polylines[poly_a_idx]
            for poly_b_idx in range(poly_a_idx + 1, len(polylines)):
                poly_b = polylines[poly_b_idx]
                
                for seg_a in poly_a.segments:
                    seg_a_geom = seg_a.get_seg_geom()
                    if seg_a_geom.length < eps:
                        continue
                    for seg_b in poly_b.segments:
                        seg_b_geom = seg_b.get_seg_geom()
                        if seg_b_geom.length < eps:
                            continue

                        inter = self._seg_intersect(seg_a_geom,seg_b_geom)
                        if inter is None:
                            continue

                        inter_pt = Point2D(inter.x, inter.y)
                        # is_seg_a_end = (inter_pt == seg_a.start) or (inter_pt == seg_a.end)
                        # is_seg_b_end = (inter_pt == seg_b.start) or (inter_pt == seg_b.end)
                        # if is_seg_a_end and is_seg_b_end:
                        #     continue

                        if inter_pt not in intersections:
                            intersections[inter_pt] = []
                        intersections[inter_pt].append({
                            'poly_idx': poly_a.poly_idx,
                            'seg_idx': seg_a.seg_idx
                        })
                        intersections[inter_pt].append({
                            'poly_idx': poly_b.poly_idx,
                            'seg_idx': seg_b.seg_idx
                        })

        # 去重
        for pt, infos in intersections.items():
            unique_infos = []
            seen_keys = set()
            for info in infos:
                key = (info['poly_idx'], info['seg_idx'])
                if key not in seen_keys:
                    seen_keys.add(key)
                    unique_infos.append(info)
            intersections[pt] = unique_infos

        return intersections

    def _insert_intersections_to_polyline(self, polyline: Polyline2D) -> List[Point2D]:
        poly_idx = polyline.poly_idx
        seg_to_inters: Dict[int, List[Point2D]] = {}
        
        for inter_pt, infos in self._intersections.items():
            for info in infos:
                if info['poly_idx'] != poly_idx:
                    continue
                seg_idx = info['seg_idx']
                if seg_idx not in seg_to_inters:
                    seg_to_inters[seg_idx] = []
                seg_to_inters[seg_idx].append(inter_pt)
                break
        
        for seg_idx in seg_to_inters:
            seg_start = polyline.points[seg_idx]
            def dist_to_start(pt: Point2D) -> float:
                return ((pt.x - seg_start.x) ** 2 + (pt.y - seg_start.y) ** 2) ** 0.5
            seg_to_inters[seg_idx].sort(key=dist_to_start)
        
        new_vertices = []
        for i in range(len(polyline.points)):
            curr_pt = polyline.points[i]
            new_vertices.append(curr_pt)
            if i < len(polyline.points) - 1 and i in seg_to_inters:
                for inter_pt in seg_to_inters[i]:
                    new_vertices.append(inter_pt)
        
        cleaned_vertices = []
        for pt in new_vertices:
            if not cleaned_vertices or not (pt == cleaned_vertices[-1]):
                cleaned_vertices.append(pt)
        
        return cleaned_vertices

    def _split_polyline_at_intersections(self, polyline: Polyline2D) -> List[Polyline2D]:
        new_vertices = self._insert_intersections_to_polyline(polyline)
        
        # 检查是否有交点在这条多段线上（包括原始顶点中的交点）
        has_intersection_on_this_poly = False
        for inter_pt in self._intersections:
            for info in self._intersections[inter_pt]:
                if info['poly_idx'] == polyline.poly_idx:
                    has_intersection_on_this_poly = True
                    break
            if has_intersection_on_this_poly:
                break
        
        # 如果没有插入新顶点且没有内部交点，则不需要拆分
        if len(new_vertices) == len(polyline.points) and not has_intersection_on_this_poly:
            return [polyline]

        sub_polylines = []
        start_idx = 0
        base_poly_idx = polyline.poly_idx

        for i in range(1, len(new_vertices)):
            curr_pt = new_vertices[i]
            is_intersection = curr_pt in self._intersections
            is_last_point = (i == len(new_vertices) - 1)
            
            if is_intersection or is_last_point:
                sub_vertices = new_vertices[start_idx:i+1]
                sub_vertices_tuples = [pt.to_tuple() for pt in sub_vertices]
                new_poly_idx = f"{base_poly_idx}_{len(sub_polylines)}"
                sub_poly = Polyline2D(sub_vertices_tuples, new_poly_idx)
                sub_polylines.append(sub_poly)
                start_idx = i

        return sub_polylines

    def _build_point_to_subpoly_map(self) -> None:
        self._point_to_subpolys = {}
        for sub_idx, sub_poly in enumerate(self._split_polylines):
            start_pt = sub_poly.start
            end_pt = sub_poly.end
            
            if start_pt not in self._point_to_subpolys:
                self._point_to_subpolys[start_pt] = []
            self._point_to_subpolys[start_pt].append(sub_idx)
            
            if end_pt not in self._point_to_subpolys:
                self._point_to_subpolys[end_pt] = []
            self._point_to_subpolys[end_pt].append(sub_idx)

    # -------------------------------------------------------------------------
    # 核心对外接口
    # -------------------------------------------------------------------------
    def run(self) -> None:
        """执行主流程：检测交点 -> 拆分多段线 -> 构建映射关系"""
        self._intersections = self._find_all_intersections()
        
        self._split_polylines = []
        for poly in self._original_polylines:
            sub_polys = self._split_polyline_at_intersections(poly)
            self._split_polylines.extend(sub_polys)
        
        self._build_point_to_subpoly_map()
        self._has_run = True

    def get_intersection_info(self) -> List[Dict]:
        """获取所有交点信息，包含坐标及连接的拆分后子多段线索引"""
        if not self._has_run:
            raise RuntimeError("请先调用 run() 方法")
        
        result = []
        for inter_pt in self._intersections.keys():
            connected_indices = self._point_to_subpolys.get(inter_pt, [])
            result.append({
                'coordinate': inter_pt.to_tuple(),
                'connected_sub_polylines': connected_indices
            })
        return result

    def get_split_polylines(self) -> List[List[Tuple[float, float]]]:
        """获取拆分后的所有子多段线数据，按照顺序排列"""
        if not self._has_run:
            raise RuntimeError("请先调用 run() 方法")
        return [[pt.to_tuple() for pt in poly.points] for poly in self._split_polylines]

    # -------------------------------------------------------------------------
    # 可选：可视化调试接口
    # -------------------------------------------------------------------------
    def visualize(self, title: str = "结果可视化") -> None:
        if not self._has_run:
            raise RuntimeError("请先调用 run() 方法")
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
        
        # 左图：原始
        ax1.set_title(f"【原始多段线】{title}", fontsize=14)
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        for idx, poly in enumerate(self._original_polylines):
            x, y = zip(*[p.to_tuple() for p in poly.points])
            ax1.plot(x, y, color=colors[idx % len(colors)], linewidth=3, label=f'原始 {idx}')
        for pt in self._intersections.keys():
            ax1.scatter(pt.x, pt.y, c='red', s=100, zorder=10, edgecolors='black')
        ax1.legend(), ax1.grid(True, linestyle='--'), ax1.axis('equal')
        
        # 右图：拆分后
        ax2.set_title(f"【拆分后多段线】{title}", fontsize=14)
        split_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        for idx, poly in enumerate(self._split_polylines):
            x, y = zip(*[p.to_tuple() for p in poly.points])
            color = split_colors[idx % len(split_colors)]
            ax2.plot(x, y, color=color, linewidth=2.5, label=f'sub line: [{idx}]')
            # 标注编号
            if len(poly.points) >= 2:
                mid_pt = poly.points[len(poly.points) // 2]
                ax2.text(mid_pt.x + 0.05, mid_pt.y + 0.05, f'[{idx}]', 
                         fontsize=11, fontweight='bold', color=color,
                         bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
        
        # 标注交点连接关系
        for inter_pt in self._intersections.keys():
            ax2.scatter(inter_pt.x, inter_pt.y, c='red', s=150, zorder=10, edgecolors='black', linewidth=2)
            conn = self._point_to_subpolys.get(inter_pt, [])
            ax2.text(inter_pt.x, inter_pt.y + 0.15, f"connected: {conn}", 
                     fontsize=10, ha='center', bbox=dict(facecolor='yellow', alpha=0.9, edgecolor='orange', boxstyle='round,pad=0.3'))
        
        ax2.legend(bbox_to_anchor=(1.02, 1), loc='upper left', fontsize=10)
        ax2.grid(True, linestyle='--'), ax2.axis('equal')
        
        # plt.tight_layout()
        # plt.show()

        outdir  = "result"
        os.makedirs(outdir,exist_ok=True)
        img_path = f"{outdir}/intersection.png"
        plt.savefig(img_path)


def lonlat_to_enu(trans_pt,ref_pt ):
    """使用pymap3d将经纬度转换为ENU坐标系"""
    lon, lat = trans_pt
    ref_lon, ref_lat = ref_pt
    x, y, _ = pymap3d.geodetic2enu(lat, lon, 0.0, ref_lat, ref_lon, 0.0)
    return [float(x), float(y)]



def load_geojson_centerlines(geojson_file: str = "data/b1.geojson", target_entity: str = "车道中心线") -> List[List[Tuple[float, float]]]:
    """从GeoJSON文件中加载指定类型的线数据"""
    with open(geojson_file, 'r') as pf:
        data = json.load(pf)
    
    lines = []
    for feature in data["features"]:
        entity_name = feature["properties"]["entity_name"]
        if feature["geometry"] is None:
            continue
        line = feature["geometry"]["coordinates"]
        if entity_name == target_entity:
            lines.append(line)

    return lines

def load_geojson_lines(geojson_file: str = "data/lines.geojson") -> List[List[Tuple[float, float]]]:
    """从GeoJSON文件中加载所有线数据"""
    with open(geojson_file, 'r') as pf:
        data = json.load(pf)
    
    lines = []
    for feature in data["features"]:
        if feature["geometry"] is None:
            continue
        if feature["geometry"]["type"] == "LineString":
            line = feature["geometry"]["coordinates"]
            lines.append(line)

    return lines


# ========== 使用示例 ==========
if __name__ == "__main__":

    

    lines = load_geojson_centerlines()
    # lines = load_geojson_lines()

    # 坐标转换,参考点为第一个点
    ref_gps =  lines[0][0]

    trans_lines = list()
    for line in lines:
        trans_coords = [lonlat_to_enu(pt,ref_gps) for pt in line]
        trans_lines.append(trans_coords)


    # 2. 初始化并运行
    finder = IntersectionFinder(trans_lines)
    finder.run()

    # 3. 【接口1】获取交点信息（含连接关系）
    print("===== 交点信息 =====")
    intersections = finder.get_intersection_info()
    for i, inter in enumerate(intersections):
        print(f"交点 {i+1}:")
        print(f"  坐标: {inter['coordinate']}")
        print(f"  连接子多段线: {inter['connected_sub_polylines']}")

    # 4. 【接口2】获取拆分后的多段线
    print("\n===== 拆分后多段线 =====")
    split_polys = finder.get_split_polylines()
    for i, poly in enumerate(split_polys):
        print(f"子多段线 [{i}]: {poly}")

    # 5. 可选：可视化
    finder.visualize()