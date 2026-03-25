""" Created at:2026/03/23 16:47:38,@Author: yimy."""
import json
import math
import time
from typing import List, Tuple, Dict, Set, Optional
from collections import defaultdict
import numpy as np
from scipy.spatial import cKDTree
from subobject import Point, Polyline
import shapefile  # 需要安装 pyshp 库
import matplotlib.pyplot as plt

class PolylineMerger:
    def __init__(self) -> None:
        self.EPS1 = 0.3        # 第一阶段：端点距离阈值（调大以便能合并更多）
        self.EPS2 = 8.0          # 第二阶段：端点距离阈值
        self.COS_THRESH = 0.7    # 方向向量夹角余弦绝对值阈值


    def __call__(self, polylines: List[Polyline]):
        """ 主函数，输入多段线列表，输出合并后的多段线列表（GeoJSON格式） """
        # step1 第一阶段：基于端点距离合并多段线，适用于端点距离较近的情况
        merged_polylines_stat1 = self.merge_polylines_stat1(polylines)
        # 可视化第一阶段的合并结果，便于调试和验证
        self.visualize_polylines(merged_polylines_stat1)

        # 检测闭合多段线
        stat1_polygons, stat1lines = self.polylines_to_polygon(merged_polylines_stat1)

        # # step2 第二阶段：基于端点距离和方向相似度合并未闭合多段线，适用于端点距离较远但方向相似的情况
        self.visualize_polylines(stat1lines,"lines_unclosed.png")

        merged_polylines_stat2 = self.merge_polylines_stat2(stat1lines) 
        self.visualize_polylines(merged_polylines_stat2,"lines_merged.png")

        stat2_polygons, stat2lines = self.polylines_to_polygon(merged_polylines_stat2)
        all_polygons = stat1_polygons + stat2_polygons
        all_lines = stat2lines

        self.visualize_polylines(all_polygons,"all_polygons.png")
        
        # # step3 将合并后的多段线转换为GeoJSON格式输出
        geojson = self.polylines_to_geojson(all_polygons)
        
        return geojson



    def merge_polylines_stat1(self, polylines: List[Polyline]) -> List[Polyline]:
        """ 第一阶段：基于端点距离合并多段线，适用于端点距离较近的情况 
            输入：多段线列表
            输出：合并后的多段线列表
        """
        
        n = len(polylines)
        
        # step1 利用多段线端点构建对象关联表，记录哪些多段线的端点距离在EPS1范围内
        Polylineobj_related_dict = defaultdict(set)
        
        for i in range(n): 
            for j in range(i + 1, n):
                pl_i = polylines[i]
                pl_j = polylines[j]
                # 判断pl_i的起点和pl_j的起点、终点的距离,且存在端点所在线段距离很近的情况，认为这两条多段线是相关的，应该合并
                if pl_i.start and pl_j.start and pl_i.start.distance_to(pl_j.start) < self.EPS1:
                    Polylineobj_related_dict[i].add(j)
                    Polylineobj_related_dict[j].add(i)
                if pl_i.start and pl_j.end and pl_i.start.distance_to(pl_j.end) < self.EPS1:
                    Polylineobj_related_dict[i].add(j)
                    Polylineobj_related_dict[j].add(i)
                if pl_i.end and pl_j.start and pl_i.end.distance_to(pl_j.start) < self.EPS1:
                    Polylineobj_related_dict[i].add(j)
                    Polylineobj_related_dict[j].add(i)
                if pl_i.end and pl_j.end and pl_i.end.distance_to(pl_j.end) < self.EPS1:
                    Polylineobj_related_dict[i].add(j)
                    Polylineobj_related_dict[j].add(i)
                else:
                    # 完全没有关联的多段线也要加入关联表，方便后续聚类
                    Polylineobj_related_dict[i].add(i)
                    Polylineobj_related_dict[j].add(j)

        # step2 对关联表进行聚类，得到关联的多段线组   
        visited = set()
        clusters = []
        for idx in Polylineobj_related_dict:
            if idx not in visited:
                cluster = []
                stack = [idx]
                while stack:
                    current = stack.pop()
                    if current not in visited:
                        visited.add(current)
                        cluster.append(current)
                        stack.extend(Polylineobj_related_dict[current])
                clusters.append(cluster)
        # step3 对每个关联的多段线组进行合并，得到新的多段线列表
        merged_polylines = []
        for cluster in clusters:
            if len(cluster) == 1:
                merged_polylines.append(polylines[cluster[0]])
            else:
                # 这里可以根据需要选择合并策略，按照第一个polyline的顺序进行合并
                merged_points = []
                for idx in cluster:
                    # 依次判断每个polyline的起点和终点与当前合并结果的关系，决定是否需要反转
                    pl = polylines[idx]
                    if not merged_points:
                        merged_points.extend(pl.points)
                    else:
                        last_point = merged_points[-1]
                        if pl.start and last_point.distance_to(pl.start) < self.EPS1:
                            merged_points.extend(pl.points[1:])  # 连接时去掉重复点
                        elif pl.end and last_point.distance_to(pl.end) < self.EPS1:
                            merged_points.extend(reversed(pl.points[:-1]))  # 连接时去掉重复点
                        else:
                            # 如果都不满足，说明这个polyline可能需要反转
                            if pl.start and merged_points[0].distance_to(pl.start) < self.EPS1:
                                merged_points = list(reversed(pl.points)) + merged_points[1:]  # 连接时去掉重复点
                            elif pl.end and merged_points[0].distance_to(pl.end) < self.EPS1:
                                merged_points = pl.points + merged_points[1:]  # 连接时去掉重复点
                            else:
                                # 如果还是不满足，说明这个polyline可能无法合并，直接添加到结果中
                                merged_polylines.append(Polyline(merged_points, closed=False))
                                merged_points = pl.points
                if merged_points:
                    merged_polylines.append(Polyline(merged_points, closed=False))
        
        return merged_polylines

    def polylines_to_polygon(self, merged_polylines_stat1: List[Polyline]):
        """将合并后闭合的多段线转换为多边形，并将未闭合的多段线保持为线段"""
        polygons = []
        lines = []
        for pl in merged_polylines_stat1:
            # 判断多段线是否闭合，如果首尾点距离小于EPS1则认为是闭合的
            if pl.start and pl.end and pl.start.distance_to(pl.end) < self.EPS1:
                pl.closed = True
            if pl.closed:
                polygons.append(pl)
            else:
                lines.append(pl)
        return polygons, lines


    def merge_polylines_stat2(self, lines: List[Polyline]):
        # step1 构建新的关联表，记录哪些多段线的端点距离在EPS2范围内且方向相似度大于COS_THRESH
        Polylineobj_candidate_dict = defaultdict(set)
        for i in range(len(lines)):
            
            for j in range(i + 1, len(lines)):
                pl_i = lines[i]
                pl_j = lines[j]
                # 判断pl_i的端点和pl_j的端点的距离和方向相似度
                if pl_i.start and pl_j.start and pl_i.start.distance_to(pl_j.start) < self.EPS2:

                    Polylineobj_candidate_dict[i].add(j)
                    Polylineobj_candidate_dict[j].add(i)

                if pl_i.start and pl_j.end and pl_i.start.distance_to(pl_j.end) < self.EPS2:

                    Polylineobj_candidate_dict[i].add(j)
                    Polylineobj_candidate_dict[j].add(i)

                if pl_i.end and pl_j.start and pl_i.end.distance_to(pl_j.start) < self.EPS2:
                    Polylineobj_candidate_dict[i].add(j)
                    Polylineobj_candidate_dict[j].add(i)

                if pl_i.end and pl_j.end and pl_i.end.distance_to(pl_j.end) < self.EPS2:
                    Polylineobj_candidate_dict[i].add(j)
                    Polylineobj_candidate_dict[j].add(i)
                else:
                    # 完全没有关联的多段线也要加入关联表，方便后续聚类
                    Polylineobj_candidate_dict[i].add(i)
                    Polylineobj_candidate_dict[j].add(j)


        # step2 对关联表进行聚类，得到候选的多段线组
        visited = set()
        clusters = []
        for idx in Polylineobj_candidate_dict:
            if idx not in visited:
                cluster = []
                stack = [idx]
                while stack:
                    current = stack.pop()
                    if current not in visited:
                        visited.add(current)
                        cluster.append(current)
                        stack.extend(Polylineobj_candidate_dict[current])
                clusters.append(cluster)

        # step3 对每个候选的多段线组进行合并，需要考虑其中的连接点，得到新的多段线列表
        merged_polylines = []
        for cluster in clusters:
            if len(cluster) == 1:
                # 看看自身能否满足闭合条件
                pl = lines[cluster[0]]
                if pl.start and pl.end and pl.start.distance_to(pl.end) < self.EPS2:
                    # 结尾保存第一个点
                    pl.points.append(pl.points[0])
                    merged_polylines.append(pl)
                else:
                    merged_polylines.append(lines[cluster[0]])
            else:
                # 这里可以根据需要选择合并策略，按照第一个polyline的顺序进行合并
                merged_points = []
                for idx in cluster:
                    # 依次判断每个polyline的起点和终点与当前合并结果的关系，决定是否需要反转
                    pl = lines[idx]
                    if not merged_points:
                        merged_points.extend(pl.points)
                    else:
                        last_point = merged_points[-1]
                        if pl.start and last_point.distance_to(pl.start) < self.EPS2:
                            # 端点需要连接起来
                            merged_points.extend(pl.points[0:])  # 连接时去掉重复点
                        elif pl.end and last_point.distance_to(pl.end) < self.EPS2:
                            merged_points.extend(reversed(pl.points))  # 连接时去掉重复点
                        else:
                            # 如果都不满足，说明这个polyline可能需要反转
                            if pl.start and merged_points[0].distance_to(pl.start) < self.EPS2:
                                merged_points = list(reversed(pl.points)) + merged_points  # 连接时去掉重复点
                            elif pl.end and merged_points[0].distance_to(pl.end) < self.EPS2:
                                merged_points = pl.points + merged_points  # 连接时去掉重复点
                            else:
                                # 如果还是不满足，说明这个polyline可能无法合并，直接添加到结果中
                                merged_polylines.append(Polyline(merged_points, closed=False))
                                merged_points = pl.points
                if merged_points:
                    polyclose = merged_points + [merged_points[0]] if merged_points[0].distance_to(merged_points[-1]) < self.EPS2 else merged_points
                    merged_polylines.append(Polyline(polyclose, closed=False))

        return merged_polylines


    def visualize_polylines(self, polylines: List[Polyline], filename="merged_polylines.png", point_size: float = 1.0) -> None:
        """ 可视化多段线列表，便于调试和验证
        根据 polyline 数量，按照红到蓝的颜色渐变显示不同的 polyline，便于区分和观察合并效果
        
        Args:
            polylines: 多段线列表
            filename: 保存图片的文件名
            point_size: 每个点的大小（像素），默认为 2
        """
        fileout = f"result_plot/{filename}"

        for i, pl in enumerate(polylines):
            color = plt.cm.jet(i / len(polylines))  # 颜色渐变
            x = [p.x for p in pl.points]
            y = [p.y for p in pl.points]
            plt.plot(x, y, marker='o', color=color, linewidth=0.5,markersize=point_size)
        plt.axis('equal')
        
        # 保存结果，大小为2160*2160，dpi为800，确保高质量输出
        plt.savefig(fileout, dpi=800, bbox_inches='tight', pad_inches=0, transparent=True)
        plt.clf()  # 清除当前图像，准备下一次绘制



    def cosine_similarity(self, vec1, vec2):
        """计算两个向量的余弦相似度"""
        dot_product = vec1[0] * vec2[0] + vec1[1] * vec2[1]
        norm_vec1 = math.sqrt(vec1[0] ** 2 + vec1[1] ** 2)
        norm_vec2 = math.sqrt(vec2[0] ** 2 + vec2[1] ** 2)
        if norm_vec1 == 0 or norm_vec2 == 0:
            return 0.0
        return dot_product / (norm_vec1 * norm_vec2)

    def polylines_to_geojson(self,polygons):
        """保存所有的polygon，区分内外环"""
        
        # step1 计算polygon面积，将最大面积作为外环，其他的作为内环

        max_area = 0
        outer_polygon = None
        for poly  in polygons:
            area = poly.calculate_area()
            if area > max_area:
                max_area = area
                outer_polygon = poly
        if outer_polygon is None:
            return {
                "type": "FeatureCollection",
                "features": []
            }
        features = []
        for poly in polygons:
            geojson = poly.to_geojson()
            # 通过属性区分内外环，外环的属性为 "is_outer": True，内环的属性为 "is_outer": False
            if poly == outer_polygon:
                geojson["properties"]["is_outer"] = True
            else:
                geojson["properties"]["is_outer"] = False
            features.append(geojson)
        return {
            "type": "FeatureCollection",
            "features": features
        }

if __name__ == "__main__":
    # 读取输入数据
    filepath = "data/B3L_enu.geojson"

    polylines = list()
    with open(filepath, 'r', encoding='utf-8') as f:
        features = json.load(f).get("features", [])
        for i, feature in enumerate(features):
            coords = feature["geometry"]["coordinates"]
            points = [Point(x, y) for x, y in coords]
            polyline = Polyline(points, closed=False, idx=i)

            if len(points) >= 2 and polyline.get_length()> 0.1:
                polylines.append(polyline)


    # 创建合并器并执行合并
    merger = PolylineMerger()
    merged_geojson = merger(polylines)
    

    # 输出结果
    with open("data/polygon_output.json", "w") as f:
        json.dump(merged_geojson, f, indent=2)