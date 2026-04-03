import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Set, Optional, Union
from shapely.geometry import LineString, Polygon, MultiPolygon
from shapely.ops import unary_union
from shapely.plotting import plot_polygon
import json,os
import pymap3d

__all__ = ["Config","GeometryUtils","IntersectionResult","PolylineIntersectionFinder","CrossBuffer"]



# ===================== 配置常量 =====================
class Config:
    """全局配置常量"""
    EPS: float = 1e-6
    DEFAULT_BUFFER_DISTANCE: float = 3.0
    LATERAL_BUFFER_WIDTH: float = 3.0
    CAP_STYLE: int = 2  # 方形端点
    JOIN_STYLE: int = 2  # 方形连接
    ANGLE_THRESHOLD: float = 10.0  # T字形判断角度阈值

# ===================== 几何工具类（完全独立） =====================
class GeometryUtils:
    """几何计算工具类（静态方法，无状态）"""
    
    @staticmethod
    def distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """计算两点之间的距离"""
        return np.linalg.norm(np.array(p1) - np.array(p2))
    
    @staticmethod
    def polyline_length(vertices: List[Tuple[float, float]]) -> float:
        """计算多段线的总长度"""
        length = 0.0
        for i in range(len(vertices) - 1):
            length += GeometryUtils.distance(vertices[i], vertices[i+1])
        return length
    
    @staticmethod
    def round_point(point: Tuple[float, float], decimals: int = 12) -> Tuple[float, float]:
        """对点坐标进行四舍五入"""
        return (round(point[0], decimals), round(point[1], decimals))
    
    @staticmethod
    def find_point_along_polyline(
        start_point: Tuple[float, float],
        vertices: List[Tuple[float, float]],
        distance: float,
        eps: float = Config.EPS
    ) -> Optional[Tuple[float, float]]:
        """
        在多段线上从起点出发，找到指定距离的点
        :param start_point: 起点（必须是多段线的端点）
        :param vertices: 多段线顶点
        :param distance: 要前进的距离
        :param eps: 容差
        :return: 找到的点，或None（如果起点不是端点）
        """
        start_arr = np.array(start_point)
        vert0_arr = np.array(vertices[0])
        vertN_arr = np.array(vertices[-1])
        
        # 判断起点在多段线的哪一端
        is_start = np.linalg.norm(start_arr - vert0_arr) < eps
        is_end = np.linalg.norm(start_arr - vertN_arr) < eps
        
        if not is_start and not is_end:
            return None
        
        # 确定遍历方向
        if is_start:
            traversal_vertices = vertices
        else:
            traversal_vertices = list(reversed(vertices))
        
        remaining_dist = distance
        current_point = np.array(traversal_vertices[0])
        
        for i in range(1, len(traversal_vertices)):
            next_point = np.array(traversal_vertices[i])
            segment_vec = next_point - current_point
            segment_len = np.linalg.norm(segment_vec)
            
            if segment_len < eps:
                continue
            
            if remaining_dist <= segment_len + eps:
                t = remaining_dist / segment_len
                buffer_point = current_point + t * segment_vec
                return (buffer_point[0], buffer_point[1])
            else:
                remaining_dist -= segment_len
                current_point = next_point
        
        # 多段线长度不足，返回最后一个顶点
        return (current_point[0], current_point[1])

# ===================== 数据传输对象（DTO） =====================
class IntersectionResult:
    """
    PolylineIntersectionFinder 的输出结果封装
    作为 CrossBuffer 的输入，实现完全解耦
    """
    def __init__(self):
        self.intersections: List[Tuple[Tuple[float, float], List[int]]] = []
        self.renumbered_polylines: List[Tuple[int, List[Tuple[float, float]]]] = []
        self.intersection_to_renumbered_ids: Dict[Tuple[float, float], List[int]] = {}
        self._intersection_set: Set[Tuple[float, float]] = set()
    
    def is_intersection_point(self, point: Tuple[float, float]) -> bool:
        """判断一个点是否是交点（纯数据查询，无外部依赖）"""
        return GeometryUtils.round_point(point) in self._intersection_set
    
    def set_intersection_set(self, intersections: List[Tuple[Tuple[float, float], List[int]]]):
        """构建交点集合"""
        self._intersection_set = set()
        for (x, y), _ in intersections:
            self._intersection_set.add(GeometryUtils.round_point((x, y)))

# ===================== 交点检测与截断类（完全独立） =====================
class PolylineIntersectionFinder:
    """
    2D多段线交点检测与截断类（完全独立）
    输出：IntersectionResult 对象
    """
    
    def __init__(self, polylines_vertices: List[List[Tuple[float, float]]], eps: float = Config.EPS):
        self.eps = eps
        self.original_polylines = self._convert_to_polylines(polylines_vertices)
        self._result = IntersectionResult()

    # ===================== 内部类 =====================

    class _Point2D:
        def __init__(self, x: float, y: float, eps: float):
            self.x = round(x, 9)
            self.y = round(y, 9)
            self._eps = eps

        def __eq__(self, other):
            if not isinstance(other, PolylineIntersectionFinder._Point2D):
                return False
            return (abs(self.x - other.x) < self._eps and
                    abs(self.y - other.y) < self._eps)

        def __hash__(self):
            return hash((self.x, self.y))

        def to_tuple(self) -> Tuple[float, float]:
            return (self.x, self.y)

    class _LineSegment2D:
        def __init__(self, start: '_Point2D', end: '_Point2D', poly_idx: int, seg_idx: int):
            self.start = start
            self.end = end
            self.poly_idx = poly_idx
            self.seg_idx = seg_idx

    class _Polyline2D:
        def __init__(self, vertices: List[Tuple[float, float]], poly_idx: int, eps: float):
            self.poly_idx = poly_idx
            self.points = [PolylineIntersectionFinder._Point2D(x, y, eps) for x, y in vertices]
            self.segments = self._split_to_segments(eps)

        def _split_to_segments(self, eps: float) -> List['_LineSegment2D']:
            segments = []
            for i in range(len(self.points) - 1):
                seg = PolylineIntersectionFinder._LineSegment2D(
                    self.points[i], self.points[i+1], self.poly_idx, i
                )
                segments.append(seg)
            return segments


    # ===================== 内部方法 =====================
    def _convert_to_polylines(self, polylines_vertices: List[List[Tuple[float, float]]]) -> List[_Polyline2D]:
        polylines = []
        for idx, vertices in enumerate(polylines_vertices):
            if len(vertices) < 2:
                raise ValueError(f"多段线 {idx} 顶点数不足2个")
            polylines.append(self._Polyline2D(vertices, idx, self.eps))
        return polylines

    @staticmethod
    def _cross_2d(v1: np.ndarray, v2: np.ndarray) -> float:
        return v1[0] * v2[1] - v1[1] * v2[0]

    def _is_point_on_segment(self, p: np.ndarray, seg_start: np.ndarray, seg_end: np.ndarray) -> bool:
        if abs(self._cross_2d(p - seg_start, seg_end - seg_start)) > self.eps:
            return False
        return (min(seg_start[0], seg_end[0]) - self.eps <= p[0] <= max(seg_start[0], seg_end[0]) + self.eps and
                min(seg_start[1], seg_end[1]) - self.eps <= p[1] <= max(seg_start[1], seg_end[1]) + self.eps)

    def _segment_intersection(self, seg1: _LineSegment2D, seg2: _LineSegment2D) -> List[Tuple[_Point2D, List[Tuple[int, int, float]]]]:
        """计算两条线段的交点"""
        A = seg1.start.to_tuple()
        B = seg1.end.to_tuple()
        C = seg2.start.to_tuple()
        D = seg2.end.to_tuple()
        
        A_arr = np.array(A)
        B_arr = np.array(B)
        C_arr = np.array(C)
        D_arr = np.array(D)

        dir1 = B_arr - A_arr
        dir2 = D_arr - C_arr
        cross_dir = self._cross_2d(dir1, dir2)
        vec_ac = C_arr - A_arr
        results = []

        if abs(cross_dir) > self.eps:
            # 不平行线段
            cross_ac_dir2 = self._cross_2d(vec_ac, dir2)
            cross_ac_dir1 = self._cross_2d(vec_ac, dir1)
            t = cross_ac_dir2 / cross_dir
            s = cross_ac_dir1 / cross_dir

            if not (0 - self.eps <= t <= 1 + self.eps and 0 - self.eps <= s <= 1 + self.eps):
                return results

            p = self._Point2D(*(A_arr + t * dir1), self.eps)
            results.append((p, [(seg1.poly_idx, seg1.seg_idx, t), (seg2.poly_idx, seg2.seg_idx, s)]))
            return results
        else:
            # 平行线段，检查端点重合
            def get_param(p: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
                dir_seg = end - start
                len_sq = np.dot(dir_seg, dir_seg)
                return np.dot(p - start, dir_seg) / len_sq if len_sq > self.eps**2 else 0.0

            if self._is_point_on_segment(A_arr, C_arr, D_arr):
                p = self._Point2D(*A, self.eps)
                results.append((p, [(seg1.poly_idx, seg1.seg_idx, 0.0), (seg2.poly_idx, seg2.seg_idx, get_param(A_arr, C_arr, D_arr))]))
            if self._is_point_on_segment(B_arr, C_arr, D_arr):
                p = self._Point2D(*B, self.eps)
                results.append((p, [(seg1.poly_idx, seg1.seg_idx, 1.0), (seg2.poly_idx, seg2.seg_idx, get_param(B_arr, C_arr, D_arr))]))
            if self._is_point_on_segment(C_arr, A_arr, B_arr) and not any(self._Point2D(*C, self.eps) == rp for rp, _ in results):
                p = self._Point2D(*C, self.eps)
                results.append((p, [(seg1.poly_idx, seg1.seg_idx, get_param(C_arr, A_arr, B_arr)), (seg2.poly_idx, seg2.seg_idx, 0.0)]))
            if self._is_point_on_segment(D_arr, A_arr, B_arr) and not any(self._Point2D(*D, self.eps) == rp for rp, _ in results):
                p = self._Point2D(*D, self.eps)
                results.append((p, [(seg1.poly_idx, seg1.seg_idx, get_param(D_arr, A_arr, B_arr)), (seg2.poly_idx, seg2.seg_idx, 1.0)]))
            return results

    # ===================== 公共方法 =====================
    def find_intersections(self) -> List[Tuple[Tuple[float, float], List[int]]]:
        """查找所有多段线之间的交点"""
        all_intersections_raw = []
        
        # 遍历所有多段线对
        for i in range(len(self.original_polylines)):
            poly1 = self.original_polylines[i]
            for j in range(i + 1, len(self.original_polylines)):
                poly2 = self.original_polylines[j]
                # 遍历所有线段对
                for seg1 in poly1.segments:
                    for seg2 in poly2.segments:
                        results = self._segment_intersection(seg1, seg2)
                        for p, seg_info in results:
                            poly_to_seg = {pi: (si, t) for pi, si, t in seg_info}
                            poly_indices = list(poly_to_seg.keys())
                            all_intersections_raw.append((p, poly_indices, poly_to_seg))

        # 去重
        unique_intersections = []
        seen_points = set()
        for p, indices, poly_to_seg in all_intersections_raw:
            if p in seen_points:
                for idx, (ep, ei, eps) in enumerate(unique_intersections):
                    if ep == p:
                        unique_intersections[idx] = (ep, list(set(ei + indices)), {**eps, **poly_to_seg})
                        break
            else:
                seen_points.add(p)
                unique_intersections.append((p, indices, poly_to_seg))

        # 保存到结果对象
        self._result.intersections = [(p.to_tuple(), sorted(indices)) for p, indices, _ in unique_intersections]
        self._result.set_intersection_set(self._result.intersections)
        
        # 保存内部细节用于截断
        self._intersection_details = unique_intersections
            
        return self._result.intersections

    def split_and_renumber_polylines(self) -> IntersectionResult:
        """在交点处截断多段线并重新编号，返回完整结果对象"""
        if not hasattr(self, '_intersection_details'):
            self.find_intersections()

        # 构建多段线到交点的映射
        poly_to_intersections: Dict[int, List[Tuple[PolylineIntersectionFinder._Point2D, int, float]]] = {}
        for p, _, poly_to_seg in self._intersection_details:
            for pi, (si, t) in poly_to_seg.items():
                if pi not in poly_to_intersections:
                    poly_to_intersections[pi] = []
                poly_to_intersections[pi].append((p, si, t))

        split_polylines_raw = []
        
        for poly in self.original_polylines:
            poly_idx = poly.poly_idx
            intersections_on_poly = poly_to_intersections.get(poly_idx, [])
            intersections_on_poly.sort(key=lambda x: (x[1], x[2]))
            intersection_set = set(p for p, _, _ in intersections_on_poly)

            # 构建完整序列
            full_sequence = [poly.points[0]]
            for seg_idx in range(len(poly.segments)):
                seg_intersections = [p for p, si, _ in intersections_on_poly if si == seg_idx]
                full_sequence.extend(seg_intersections)
                full_sequence.append(poly.points[seg_idx + 1])

            # 提取分割点
            split_points = []
            for point in full_sequence:
                if (point == full_sequence[0] or 
                    point == full_sequence[-1] or 
                    point in intersection_set):
                    if not split_points or point != split_points[-1]:
                        split_points.append(point)

            # 生成截断后的多段线
            if len(split_points) < 2:
                split_polylines_raw.append([p.to_tuple() for p in poly.points])
            else:
                for i in range(len(split_points) - 1):
                    s_start, s_end = split_points[i], split_points[i+1]
                    start_idx = full_sequence.index(s_start)
                    end_idx = full_sequence.index(s_end, start_idx)
                    new_poly = [p.to_tuple() for p in full_sequence[start_idx:end_idx+1]]
                    split_polylines_raw.append(new_poly)

        # 重新编号并保存到结果对象
        self._result.renumbered_polylines = list(enumerate(split_polylines_raw))
        
        # 建立交点到截断后多段线ID的映射
        self._map_intersections_to_renumbered_ids()
        
        return self._result

    def _map_intersections_to_renumbered_ids(self):
        """建立交点到截断后多段线ID的映射"""
        if not self._result.renumbered_polylines or not self._result.intersections:
            return

        for (x, y), _ in self._result.intersections:
            p = self._Point2D(x, y, self.eps)
            related_new_ids = []
            for new_id, vertices in self._result.renumbered_polylines:
                start_p = self._Point2D(*vertices[0], self.eps)
                end_p = self._Point2D(*vertices[-1], self.eps)
                if p == start_p or p == end_p:
                    related_new_ids.append(new_id)
            self._result.intersection_to_renumbered_ids[(x, y)] = sorted(related_new_ids)

    def get_result(self) -> IntersectionResult:
        """获取完整结果对象"""
        if not self._result.renumbered_polylines:
            self.split_and_renumber_polylines()
        return self._result

    def visualize(self, show_renumbered: bool = False, save_dir:str = "result", figsize: Tuple[int, int] = (20, 20)):
        """可视化多段线和交点"""
        fig, ax = plt.subplots(figsize=figsize)

        if show_renumbered:
            if not self._result.renumbered_polylines:
                self.split_and_renumber_polylines()
            polylines_to_show = self._result.renumbered_polylines
            title = 'Renumbered 2D Polylines (0-based) and Intersections'
        else:
            polylines_to_show = [(poly.poly_idx, [p.to_tuple() for p in poly.points]) 
                               for poly in self.original_polylines]
            title = 'Original 2D Polylines and Intersections'

        colors = plt.cm.tab20(np.linspace(0, 1, len(polylines_to_show)))
        for idx, (pid, vertices) in enumerate(polylines_to_show):
            x, y = zip(*vertices)
            ax.plot(x, y, color=colors[idx], linewidth=2, label=f'Poly {pid}')
            ax.scatter(x, y, color=colors[idx], s=50, edgecolor='k', zorder=5)
            ax.text(x[0]+0.3, y[0]+0.3, f'ID {pid}', fontsize=10, 
                   color=colors[idx], fontweight='bold', zorder=6)

        if self._result.intersections:
            for (x, y), original_ids in self._result.intersections:
                ax.scatter(x, y, color='red', s=200, marker='*', edgecolor='k', zorder=10)
                # if show_renumbered and (x, y) in self._result.intersection_to_renumbered_ids:
                #     new_ids = self._result.intersection_to_renumbered_ids[(x, y)]
                #     label = f'Poly IDs: {", ".join(map(str, new_ids))}'
                # else:
                #     label = f'Orig IDs: {", ".join(map(str, original_ids))}'
                # ax.text(x+0.3, y+0.3, label, fontsize=10, color='darkred', zorder=11,
                #        bbox=dict(facecolor='white', alpha=0.95, edgecolor='none', pad=4))

        ax.set_xlabel('X', fontsize=12)
        ax.set_ylabel('Y', fontsize=12)
        ax.set_title(title, fontsize=14, pad=20)
        # ax.legend(fontsize=9, loc='upper right', bbox_to_anchor=(1.18, 1))
        ax.grid(True, linestyle='--', alpha=1.0)
        ax.set_aspect('equal', adjustable='box')
        plt.tight_layout()
        # plt.show()

        os.makedirs(save_dir, exist_ok=True)
        plt.savefig(f'{save_dir}/intersection_visualization.png', dpi=600)
        plt.close(fig)
        ax.clear()

# ===================== 缓冲线段与Buffer生成类（完全独立） =====================
class CrossBuffer:
    """
    交点缓冲线段生成与形状分类类（完全独立）
    输入：IntersectionResult 对象（纯数据）
    """
    
    def __init__(self, 
                 intersection_result: IntersectionResult,
                 default_buffer_distance: float = Config.DEFAULT_BUFFER_DISTANCE,
                 lateral_buffer_width: float = Config.LATERAL_BUFFER_WIDTH,
                 eps: float = Config.EPS):
        """
        构造函数：只接收纯数据对象，不接收任何函数
        :param intersection_result: PolylineIntersectionFinder 的输出结果
        """
        # 完全解耦：只保存数据，不保存任何外部类的引用或函数
        self.intersections = intersection_result.intersections
        self.renumbered_polylines_dict = {pid: vertices for pid, vertices in intersection_result.renumbered_polylines}
        self.intersection_to_renumbered_ids = intersection_result.intersection_to_renumbered_ids
        self.is_intersection_point = intersection_result.is_intersection_point  # 这是数据对象的方法，不是外部类的
        
        self.default_buffer_distance = default_buffer_distance
        self.lateral_buffer_width = lateral_buffer_width
        self.lateral_buffer_half_width = lateral_buffer_width / 2
        self.eps = eps
        
        # 结果存储
        self.buffer_segments: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Tuple[float, float], float]]] = {}
        self.intersection_shapes: Dict[Tuple[float, float], str] = {}
        self.merged_buffers: Dict[Tuple[float, float], Union[Polygon, MultiPolygon]] = {}

    def generate_buffer_segments(self, debug: bool = False) -> Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Tuple[float, float]]]]:
        """
        生成所有交点的缓冲线段（单条线段独立判断）
        """
        if debug:
            print("\n" + "="*80)
            print("缓冲线段生成调试信息")
            print("="*80)

        for (inter_x, inter_y), _ in self.intersections:
            intersection = (inter_x, inter_y)
            if intersection not in self.intersection_to_renumbered_ids:
                continue

            poly_ids = self.intersection_to_renumbered_ids[intersection]
            current_buffer_segments = []

            if debug:
                print(f"\n【交点 {intersection}】")
                print(f"关联截断后多段线ID：{poly_ids}")

            for pid in poly_ids:
                if pid not in self.renumbered_polylines_dict:
                    continue
                poly_vertices = self.renumbered_polylines_dict[pid]
                poly_length = GeometryUtils.polyline_length(poly_vertices)

                # 确定线段另一端
                inter_arr = np.array(intersection)
                start_arr = np.array(poly_vertices[0])
                end_arr = np.array(poly_vertices[-1])
                if np.linalg.norm(inter_arr - start_arr) < self.eps:
                    other_end = poly_vertices[-1]
                else:
                    other_end = poly_vertices[0]

                other_end_is_intersection = self.is_intersection_point(other_end)

                # 确定缓冲距离
                if poly_length > self.default_buffer_distance:
                    buffer_dist = self.default_buffer_distance
                    rule = "规则1：长度>3，使用默认距离3"
                else:
                    if other_end_is_intersection:
                        buffer_dist = poly_length / 2
                        rule = "规则2：长度≤3，另一端是交点，使用长度的一半"
                    else:
                        buffer_dist = poly_length
                        rule = "规则3：长度≤3，另一端是普通端点，使用线段全长"

                # 生成缓冲点
                buffer_point = GeometryUtils.find_point_along_polyline(
                    intersection, poly_vertices, buffer_dist, self.eps
                )
                
                if buffer_point is not None:
                    current_buffer_segments.append((intersection, buffer_point, buffer_dist))

                    if debug:
                        print(f"  ├─ 多段线 {pid}：长度={poly_length:.2f}，另一端={other_end}，另一端是交点={other_end_is_intersection}")
                        print(f"  │  缓冲距离={buffer_dist:.2f}，缓冲点={buffer_point}，匹配{rule}")

            self.buffer_segments[intersection] = current_buffer_segments

        # 分类形状
        self._classify_shapes()
        
        # 转换为对外的简化格式
        public_buffer_segments = {}
        for inter, segs in self.buffer_segments.items():
            public_buffer_segments[inter] = [(s, e) for s, e, d in segs]
        return public_buffer_segments

    def generate_and_merge_buffers(self, debug: bool = False) -> Dict[Tuple[float, float], Union[Polygon, MultiPolygon]]:
        """
        对缓冲线段创建横向Buffer并使用unary_union合并
        """
        if not self.buffer_segments:
            self.generate_buffer_segments()

        if debug:
            print("\n" + "="*80)
            print("缓冲线段横向Buffer生成与合并调试信息")
            print("="*80)
            print(f"横向Buffer总宽度：{self.lateral_buffer_width}m（左右各{self.lateral_buffer_half_width}m）")

        self.merged_buffers = {}

        for (inter_x, inter_y), segments in self.buffer_segments.items():
            intersection = (inter_x, inter_y)
            
            # 收集该交点的所有缓冲线段并创建Buffer
            individual_buffers = []
            for (p1, p2, dist) in segments:
                line = LineString([p1, p2])
                buffer_poly = line.buffer(
                    distance=self.lateral_buffer_half_width,
                    cap_style=Config.CAP_STYLE,
                    join_style=Config.JOIN_STYLE
                )
                individual_buffers.append(buffer_poly)

            if not individual_buffers:
                continue

            # 合并Buffer
            if len(individual_buffers) == 1:
                merged_buffer = individual_buffers[0]
            else:
                merged_buffer = unary_union(individual_buffers)

            self.merged_buffers[intersection] = merged_buffer

            # if debug:
            #     print(f"\n【交点 {intersection}】")
            #     print(f"  缓冲线段数量：{len(segments)}")
            #     for idx, (p1, p2, dist) in enumerate(segments):
            #         print(f"    线段 {idx+1}：{p1} → {p2} (长度={dist:.2f})")
            #     print(f"  合并后类型：{type(merged_buffer).__name__}")
            #     if hasattr(merged_buffer, 'area'):
            #         print(f"  合并后总面积：{merged_buffer.area:.2f}")

        return self.merged_buffers

    def _classify_shapes(self):
        """内部方法：分类交点形状"""
        for (x, y), segments in self.buffer_segments.items():
            n = len(segments)
            if n == 4:
                self.intersection_shapes[(x, y)] = "Cross"
            elif n == 3:
                is_t_shape = False
                vectors = []
                for (p1, p2, _) in segments:
                    vec = np.array(p2) - np.array(p1)
                    vec_len = np.linalg.norm(vec)
                    if vec_len > self.eps:
                        vec = vec / vec_len
                    vectors.append(vec)

                for i in range(3):
                    for j in range(i+1, 3):
                        dot = np.dot(vectors[i], vectors[j])
                        dot = np.clip(dot, -1.0, 1.0)
                        angle = np.arccos(dot) * 180 / np.pi
                        if abs(angle - 180) < Config.ANGLE_THRESHOLD:
                            is_t_shape = True
                            break
                    if is_t_shape:
                        break

                self.intersection_shapes[(x, y)] = "T" if is_t_shape else "Y"
            elif n == 2:
                self.intersection_shapes[(x, y)] = "Line"
            else:
                self.intersection_shapes[(x, y)] = f"Complex({n})"

    def visualize(self, 
                  background_polylines: Optional[List[Tuple[int, List[Tuple[float, float]]]]] = None,
                  show_merged_buffers: bool = True,
                  figsize: Tuple[int, int] = (14, 10),
                  save_dir:str = "result"

    ):
        """
        可视化缓冲线段、形状分类和合并后的Buffer
        """
        if not self.buffer_segments:
            self.generate_buffer_segments()
        if show_merged_buffers and not self.merged_buffers:
            self.generate_and_merge_buffers()

        fig, ax = plt.subplots(figsize=figsize)

        # 绘制背景多段线
        if background_polylines is not None:
            colors = plt.cm.tab20(np.linspace(0, 1, len(background_polylines)))
            for idx, (pid, vertices) in enumerate(background_polylines):
                x, y = zip(*vertices)
                ax.plot(x, y, color=colors[idx], linewidth=1, alpha=0.3, label=f'Background Poly {pid}')
                ax.scatter(x, y, color=colors[idx], s=20, alpha=0.3, zorder=1)

        # 绘制合并后的Buffer
        if show_merged_buffers and self.merged_buffers:
            merged_colors = plt.cm.Set2(np.linspace(0, 1, len(self.merged_buffers)))
            for idx, (intersection, merged_buffer) in enumerate(self.merged_buffers.items()):
                plot_polygon(merged_buffer, ax=ax, add_points=False,
                            facecolor=merged_colors[idx], alpha=0.5,
                            edgecolor=merged_colors[idx], linewidth=2,
                            label=f'Merged Buffer at {intersection}')

        # 绘制缓冲线段
        buffer_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        for (x, y), segments in self.buffer_segments.items():
            ax.scatter(x, y, color='red', s=250, marker='*', edgecolor='k', zorder=10)
            for idx, (p1, p2, dist) in enumerate(segments):
                color = buffer_colors[idx % len(buffer_colors)]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, linewidth=3, zorder=5)
                ax.scatter(p2[0], p2[1], color=color, s=80, edgecolor='k', zorder=6)
                mid_x = (p1[0] + p2[0]) / 2
                mid_y = (p1[1] + p2[1]) / 2
                ax.text(mid_x + 0.2, mid_y + 0.2, f'd={dist:.2f}', 
                       fontsize=8, color=color, fontweight='bold', zorder=7)
            # if (x, y) in self.intersection_shapes:
            #     shape = self.intersection_shapes[(x, y)]
            #     n_segs = len(segments)
            #     ax.text(x + 0.4, y + 0.4, f'{shape} ({n_segs} segs)', 
            #            fontsize=11, color='darkblue', fontweight='bold', zorder=11,
            #            bbox=dict(facecolor='white', alpha=0.9, edgecolor='darkblue', pad=5))

        ax.set_xlabel('X', fontsize=12)
        ax.set_ylabel('Y', fontsize=12)
        title = f'Buffer Segments & Merged Lateral Buffers (Width={self.lateral_buffer_width}m)'
        ax.set_title(title, fontsize=14, pad=20)
        ax.legend(fontsize=8, loc='upper right', bbox_to_anchor=(1.25, 1))
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_aspect('equal', adjustable='box')
        plt.tight_layout()
        # plt.show()
        os.makedirs(save_dir, exist_ok=True)
        plt.savefig(f'{save_dir}/cross_buffer_visualization.png', dpi=800)

        plt.close(fig)
        ax.clear()



# ===================== 主函数 =====================
def main():

    # 加载geojson
    file_path = 'b1.geojson'  # 替换为你的文件路径
    with open(file_path, 'r') as f:
        geojson_data = json.load(f)

    # 找到中心线
    polylines_input_lonlat = []
    for feature in geojson_data['features']:
        if feature['properties']['entity_name'] == '车道中心线':
            vertices = feature['geometry']['coordinates']
            polylines_input_lonlat.append(vertices)

    # 以第一个点为基准，转换成enu坐标系
    polylines_input = list()
    if polylines_input_lonlat:
        ref_lon, ref_lat = polylines_input_lonlat[0][0]
        for i in range(len(polylines_input_lonlat)):
            trans_result = [lonlat_to_enu(lon, lat, ref_lon, ref_lat) for lon, lat in polylines_input_lonlat[i]]
            polylines_input.append(trans_result)

    else:
        print("未找到车道中心线数据，使用测试数据")


    # # 测试数据
    # polylines_input = [
    #     [(0,0),(4,5),(8,5),(12,12)],
    #     [(4,5),(4,10)],
    #     [(2,8),(8,8)]
    # ]

    # ===================== 第一阶段：PolylineIntersectionFinder（独立运行） =====================
    print("\n【第一阶段】PolylineIntersectionFinder 独立运行")
    print("-" * 80)
    
    # 1. 初始化并运行
    finder = PolylineIntersectionFinder(polylines_input)
    intersections = finder.find_intersections()
    
    # 2. 获取完整结果对象（纯数据）
    result = finder.split_and_renumber_polylines()
    
    # 3. 可视化
    print("\n正在显示截断后的多段线...")
    finder.visualize(show_renumbered=False)

    # ===================== 第二阶段：CrossBuffer（独立运行，只接收数据） =====================
    print("\n【第二阶段】CrossBuffer 独立运行（只接收数据对象）")
    print("-" * 80)
    
    # 4. 初始化 CrossBuffer（只传入数据对象 result，完全不知道 finder 的存在）
    cross_buffer = CrossBuffer(intersection_result=result)
    
    # 5. 生成缓冲线段
    buffer_segments = cross_buffer.generate_buffer_segments(debug=True)
    
    # 6. 创建横向Buffer并合并
    merged_buffers = cross_buffer.generate_and_merge_buffers(debug=True)
    
    # 7. 输出结果
    print("\n【结果】形状分类：")
    for (x, y), shape in cross_buffer.intersection_shapes.items():
        print(f"  交点 ({x}, {y})：{shape}")

    # 8. 可视化
    print("\n正在显示缓冲线段与合并后的Buffer...")
    cross_buffer.visualize(background_polylines=result.renumbered_polylines, show_merged_buffers=True)

    print("\n" + "="*80)
    print("完成！两个类完全独立，只通过数据对象 IntersectionResult 传递信息")
    print("="*80)

if __name__ == "__main__":
    main()