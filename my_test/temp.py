from typing import List, Tuple, Dict, Set
from shapely import Point, LineString, Polygon
from shapely.ops import unary_union
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
import os
# ========== 基础几何类（保持不变） ==========
class Point2D:
    def __init__(self, x: float, y: float, eps: float = 1e-6):
        self.x = round(x, 9)
        self.y = round(y, 9)
        self._eps = eps

    def __eq__(self, other):
        return (abs(self.x - other.x) < self._eps and
                abs(self.y - other.y) < self._eps)

    def __hash__(self):
        return hash((self.x, self.y))

    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def get_point_geom(self):
        return Point(self.x, self.y)

    def __repr__(self):
        return f"Point2D({self.x:.6f}, {self.y:.6f})"


class Segment2D:
    def __init__(self, start: Point2D, end: Point2D, poly_idx: int, seg_idx: int):
        self.start = start
        self.end = end
        self.poly_idx = poly_idx
        self.seg_idx = seg_idx

    def get_seg_geom(self):
        linecoords = [(self.start.x, self.start.y), (self.end.x, self.end.y)]
        return LineString(linecoords)

    def __repr__(self):
        return f"Segment2D(poly_idx={self.poly_idx}, seg_idx={self.seg_idx})"


class Polyline2D:
    def __init__(self, vertices: List[Tuple[float, float]], poly_idx: int):
        self.poly_idx = poly_idx
        self.points = [Point2D(x, y) for x, y in vertices]
        self.segments = self._split_to_segments()
        self.start = self.points[0]
        self.end = self.points[-1]
    
    def _split_to_segments(self) -> List[Segment2D]:
        segments = []
        for i in range(len(self.points) - 1):
            seg = Segment2D(
                self.points[i], self.points[i+1], self.poly_idx, i
            )
            segments.append(seg)
        return segments

    def get_line_geom(self):
        linecoords = [(pt.x, pt.y) for pt in self.points]
        return LineString(linecoords)

    def __repr__(self):
        return f"Polyline2D(poly_idx={self.poly_idx}, vertices={[p.to_tuple() for p in self.points]})"


# ========== IntersectionFinder 类（保持不变） ==========
class IntersectionFinder:
    def __init__(self, input_polylines: List[List[Tuple[float, float]]]):
        self._input_vertices = input_polylines
        self._original_polylines: List[Polyline2D] = []
        for idx, vertices in enumerate(input_polylines):
            if len(vertices) < 2:
                raise ValueError(f"多段线 {idx} 顶点数不足2")
            self._original_polylines.append(Polyline2D(vertices, poly_idx=idx))
        
        self._intersections: Dict[Point2D, List[Dict]] = {}
        self._split_polylines: List[Polyline2D] = []
        self._point_to_subpolys: Dict[Point2D, List[int]] = {}
        self._has_run = False

    def _find_all_intersections(self) -> Dict[Point2D, List[Dict]]:
        intersections = {}
        eps = 1e-6
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

                        inter = seg_a_geom.intersection(seg_b_geom)
                        if inter.geom_type != 'Point':
                            continue

                        inter_pt = Point2D(inter.x, inter.y)
                        is_seg_a_end = (inter_pt == seg_a.start) or (inter_pt == seg_a.end)
                        is_seg_b_end = (inter_pt == seg_b.start) or (inter_pt == seg_b.end)
                        if is_seg_a_end and is_seg_b_end:
                            continue

                        if inter_pt not in intersections:
                            intersections[inter_pt] = []
                        intersections[inter_pt].append({'poly_idx': poly_a.poly_idx, 'seg_idx': seg_a.seg_idx})
                        intersections[inter_pt].append({'poly_idx': poly_b.poly_idx, 'seg_idx': seg_b.seg_idx})

        for pt, infos in intersections.items():
            unique_infos, seen_keys = [], set()
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
                if info['poly_idx'] != poly_idx: continue
                seg_idx = info['seg_idx']
                if seg_idx not in seg_to_inters: seg_to_inters[seg_idx] = []
                seg_to_inters[seg_idx].append(inter_pt)
                break
        
        for seg_idx in seg_to_inters:
            seg_start = polyline.points[seg_idx]
            def dist(pt): return ((pt.x-seg_start.x)**2 + (pt.y-seg_start.y)**2)**0.5
            seg_to_inters[seg_idx].sort(key=dist)
        
        new_vertices = []
        for i in range(len(polyline.points)):
            new_vertices.append(polyline.points[i])
            if i < len(polyline.points)-1 and i in seg_to_inters:
                new_vertices.extend(seg_to_inters[i])
        
        cleaned = []
        for pt in new_vertices:
            if not cleaned or not (pt == cleaned[-1]): cleaned.append(pt)
        return cleaned

    def _split_polyline_at_intersections(self, polyline: Polyline2D) -> List[Polyline2D]:
        new_vertices = self._insert_intersections_to_polyline(polyline)
        if len(new_vertices) == len(polyline.points): return [polyline]

        sub_polylines = []
        start_idx = 0
        for i in range(1, len(new_vertices)):
            curr_pt = new_vertices[i]
            if (curr_pt in self._intersections) or (i == len(new_vertices)-1):
                sub_verts = new_vertices[start_idx:i+1]
                sub_poly = Polyline2D([p.to_tuple() for p in sub_verts], f"{polyline.poly_idx}_{len(sub_polylines)}")
                sub_polylines.append(sub_poly)
                start_idx = i
        return sub_polylines

    def _build_point_to_subpoly_map(self):
        self._point_to_subpolys = {}
        for idx, poly in enumerate(self._split_polylines):
            s, e = poly.start, poly.end
            if s not in self._point_to_subpolys: self._point_to_subpolys[s] = []
            self._point_to_subpolys[s].append(idx)
            if e not in self._point_to_subpolys: self._point_to_subpolys[e] = []
            self._point_to_subpolys[e].append(idx)

    def run(self):
        self._intersections = self._find_all_intersections()
        self._split_polylines = []
        for p in self._original_polylines:
            self._split_polylines.extend(self._split_polyline_at_intersections(p))
        self._build_point_to_subpoly_map()
        self._has_run = True

    def get_intersection_info(self) -> List[Dict]:
        if not self._has_run: raise RuntimeError("先 run()")
        res = []
        for pt in self._intersections:
            res.append({
                'coordinate': pt.to_tuple(),
                'connected_sub_polylines': self._point_to_subpolys.get(pt, [])
            })
        return res

    def get_split_polylines(self) -> List[List[Tuple[float, float]]]:
        if not self._has_run: raise RuntimeError("先 run()")
        return [[p.to_tuple() for p in poly.points] for poly in self._split_polylines]
    
    # 新增：供 CrossJunction 使用的内部接口（获取所有交点坐标集合）
    def _get_all_intersection_coords_set(self) -> Set[Tuple[float, float]]:
        return {pt.to_tuple() for pt in self._intersections.keys()}

    def visualize(self, title="可视化"):
        if not self._has_run: raise RuntimeError("先 run()")
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
        
        # 左
        ax1.set_title(f"原始 {title}")
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
        for i, p in enumerate(self._original_polylines):
            x, y = zip(*[pt.to_tuple() for pt in p.points])
            ax1.plot(x, y, color=colors[i%3], lw=3, label=f'原始 {i}')
        for pt in self._intersections: ax1.scatter(pt.x, pt.y, c='red', s=100, zorder=10)
        ax1.legend(), ax1.grid(True), ax1.axis('equal')
        
        # 右
        ax2.set_title(f"拆分后 {title}")
        split_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        for i, p in enumerate(self._split_polylines):
            x, y = zip(*[pt.to_tuple() for pt in p.points])
            ax2.plot(x, y, color=split_colors[i%5], lw=2, label=f'[{i}]')
        for pt in self._intersections: ax2.scatter(pt.x, pt.y, c='red', s=100, zorder=10)
        ax2.legend(bbox_to_anchor=(1.02, 1)), ax2.grid(True), ax2.axis('equal')
        # plt.tight_layout()
        # plt.show()

        outdir  = "result"
        os.makedirs(outdir,exist_ok=True)
        img_path = f"{outdir}/intersc.png"
        plt.savefig(img_path)



# ========== 【新增】CrossJunction 类 ==========

class CrossJunction:
    def __init__(self, 
                 intersection_info: List[Dict], 
                 split_polylines: List[List[Tuple[float, float]]],
                 all_intersection_coords: Set[Tuple[float, float]]):
        self._inter_info = intersection_info
        self._split_polys = split_polylines
        self._all_inter_coords = all_intersection_coords
        self._junction_results: List[Dict] = []

    def run(self, target_distance: float = 1.5, buffer_width: float = 1.0):
        self._junction_results = []
        
        for info in self._inter_info:
            junc_coord = info['coordinate']
            connected_indices = info['connected_sub_polylines']
            
            if len(connected_indices) <= 2:
                continue
            
            buffer_geoms = []
            target_points = []
            
            for poly_idx in connected_indices:
                poly_verts = self._split_polys[poly_idx]
                line_geom = LineString(poly_verts)
                
                start_coord = poly_verts[0]
                end_coord = poly_verts[-1]
                
                # ===========================================================
                # 【关键修正 1】：明确判断方向，并提取从路口出发的射线
                # ===========================================================
                is_start_junc = (start_coord == junc_coord)
                is_end_junc = (end_coord == junc_coord)
                
                if not (is_start_junc or is_end_junc):
                    continue # 理论上不会发生
                
                # 构建“从路口指向另一端”的坐标列表
                if is_start_junc:
                    # 情况A：路口在起点 -> 直接用原坐标
                    coords_from_junc = poly_verts
                else:
                    # 情况B：路口在终点 -> 反转坐标列表！
                    coords_from_junc = list(reversed(poly_verts))
                
                # 基于新的坐标列表构建 LineString（确保起点是路口）
                line_from_junc = LineString(coords_from_junc)
                
                # ===========================================================
                # 【关键修正 2】：判断另一端是否也是交点
                # ===========================================================
                # 注意：现在 coords_from_junc[0] 是路口，coords_from_junc[-1] 是另一端
                other_end_coord = coords_from_junc[-1]
                other_end_is_junc = (other_end_coord in self._all_inter_coords)
                
                # ===========================================================
                # 计算目标距离（现在可以放心地从 0 开始算了）
                # ===========================================================
                if other_end_is_junc:
                    # 两端都是交点 -> 取中点
                    target_dist = line_from_junc.length / 2.0
                else:
                    # 普通情况 -> 取 1.5，或者线长的 99%（防止超出）
                    target_dist = min(target_distance, line_from_junc.length * 0.99)
                
                # ===========================================================
                # 安全插值（起点一定是路口）
                # ===========================================================
                junc_point = Point(junc_coord)
                target_point = line_from_junc.interpolate(target_dist)
                
                # ===========================================================
                # 构建 Buffer
                # ===========================================================
                # 为了保险，重新构建中心线（虽然 line_from_junc 前一段就是）
                center_line = LineString([junc_point, target_point])
                buffer_poly = center_line.buffer(buffer_width, cap_style=1)
                
                buffer_geoms.append(buffer_poly)
                target_points.append(target_point)
            
            # 合并
            if buffer_geoms:
                merged_poly = unary_union(buffer_geoms)
                self._junction_results.append({
                    'junction_coordinate': junc_coord,
                    'connected_poly_indices': connected_indices,
                    'target_points': [(p.x, p.y) for p in target_points],
                    'merged_polygon': merged_poly
                })

    def get_junction_polygons(self) -> List[Dict]:
        return self._junction_results

    def visualize(self, original_split_polylines: List[List[Tuple[float, float]]], title: str = "路口缓冲区分析"):
        fig, ax = plt.subplots(1, 1, figsize=(12, 12))
        
        # 底图
        for poly in original_split_polylines:
            x, y = zip(*poly)
            ax.plot(x, y, color='lightgray', linewidth=2, linestyle='--', label='_nolegend_')
        
        # 绘制结果
        colors = ['#ff7f0e', '#1f77b4', '#2ca02c', '#d62728']
        for idx, res in enumerate(self._junction_results):
            color = colors[idx % len(colors)]
            poly = res['merged_polygon']
            
            # 绘制多边形
            if poly.geom_type == 'Polygon':
                x, y = poly.exterior.xy
                ax.fill(x, y, color=color, alpha=0.5, label=f'路口 {idx+1}')
                ax.plot(x, y, color=color, linewidth=2)
            elif poly.geom_type == 'MultiPolygon':
                for p in poly.geoms:
                    x, y = p.exterior.xy
                    ax.fill(x, y, color=color, alpha=0.5)
                    ax.plot(x, y, color=color, linewidth=2)
            
            # 绘制中心点和目标点
            jx, jy = res['junction_coordinate']
            ax.scatter(jx, jy, color='black', s=120, zorder=10, marker='X', label='_nolegend_')
            
            tps = res['target_points']
            if tps:
                tpx, tpy = zip(*tps)
                ax.scatter(tpx, tpy, color=color, s=80, zorder=9, edgecolor='black', label='_nolegend_')
        
        ax.set_title(title, fontsize=16)
        ax.legend(fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.axis('equal')
        # plt.tight_layout()
        # plt.show()
        outdir  = "result"
        os.makedirs(outdir,exist_ok=True)
        img_path = f"{outdir}/junc.png"
        plt.savefig(img_path)





# ========== 测试用例 ==========
if __name__ == "__main__":
    # 1. 准备测试数据：一个十字路口（4条路交汇）+ 一个T字路口（3条路交汇）
    # 结构：
    #   十字路口 (0,0): 连接上下左右
    #   T字路口  (5,0): 连接右、上、下
    input_data = [
        # 横线（穿过十字路口）
        [(-3, 0), (0, 0), (5, 0), (8, 0)],
        # 竖线1（十字路口南北）
        [(0, 3), (0, 0), (0, -3)],
        # 竖线2（T字路口南北）
        [(5, 3), (5, 0), (5, -3)]
    ]

    # 2. 运行 IntersectionFinder
    print("Step 1: 运行 IntersectionFinder...")
    finder = IntersectionFinder(input_data)
    finder.run()
    
    # 获取结果
    inter_info = finder.get_intersection_info()
    split_polys = finder.get_split_polylines()
    all_inter_coords = finder._get_all_intersection_coords_set() # 获取所有交点坐标集合

    # 3. 运行 CrossJunction
    print("\nStep 2: 运行 CrossJunction...")
    junction = CrossJunction(inter_info, split_polys, all_inter_coords)
    junction.run(target_distance=1.5, buffer_width=1.0)
    
    # 4. 获取并打印结果
    results = junction.get_junction_polygons()
    print(f"\n找到 {len(results)} 个多岔路口（连接数>2）：")
    for i, res in enumerate(results):
        print(f"  路口 {i+1}: 坐标 {res['junction_coordinate']}")
        print(f"    连接子多段线数量: {len(res['connected_poly_indices'])}")
        print(f"    生成目标点数量: {len(res['target_points'])}")

    # 5. 可视化
    print("\nStep 3: 可视化...")
    # 先看拆分结果
    finder.visualize("多段线拆分结果")
    # 再看路口缓冲区
    junction.visualize(split_polys, title="多岔路口缓冲区合并结果")