# -*- coding: utf-8 -*-

from __future__ import unicode_literals

import concurrent.futures
from typing import List, Union, Optional
import json
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import (
    LineString,
    MultiLineString,
    MultiPolygon,
    Polygon,
    box,
)

from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.strtree import STRtree


class InvalidInputTypeError(Exception):
    pass


class TooFewRidgesError(Exception):
    pass


class Centerline:
    """从单个多边形（含孔）生成中心线。"""

    def __init__(
        self,
        input_geometry: Union[Polygon, MultiPolygon],
        interpolation_distance: Optional[float] = None,
        max_points: int = 10000,
        **attributes,
    ):
        """
        :param input_geometry: 输入几何（Polygon 或 MultiPolygon）
        :param interpolation_distance: 边界插值点间距（若为 None，则自适应）
        :param max_points: 最大边界点数（防止内存爆炸）
        :param attributes: 附加属性
        """
        self._input_geometry = input_geometry
        self._max_points = max_points

        # 自适应插值距离：使总点数不超过 max_points
        if interpolation_distance is None:
            total_boundary_length = sum(
                poly.length for poly in self._extract_polygons_from_input_geometry()
            )
            # 确保每个边界至少 3 个点，防止除零
            if total_boundary_length > 0:
                self._interpolation_distance = max(
                    total_boundary_length / max_points, 1e-6
                )
            else:
                self._interpolation_distance = 0.5
        else:
            self._interpolation_distance = abs(interpolation_distance)

        if not self.input_geometry_is_valid():
            raise InvalidInputTypeError("Input must be Polygon or MultiPolygon")

        self._min_x, self._min_y = self._get_reduced_coordinates()
        self.assign_attributes_to_instance(attributes)

        self.geometry = MultiLineString(lines=self._construct_centerline())

    def input_geometry_is_valid(self) -> bool:
        return isinstance(self._input_geometry, (Polygon, MultiPolygon))

    def _get_reduced_coordinates(self) -> tuple:
        """获取平移量（几何最小坐标取整）"""
        min_x = int(min(self._input_geometry.envelope.exterior.xy[0]))
        min_y = int(min(self._input_geometry.envelope.exterior.xy[1]))
        return min_x, min_y

    def assign_attributes_to_instance(self, attributes: dict):
        for key, value in attributes.items():
            setattr(self, key, value)

    def _construct_centerline(self) -> MultiLineString:
        """构建中心线的主方法"""
        vertices, ridges = self._get_voronoi_vertices_and_ridges()
        linestrings = []

        for ridge in ridges:
            if self._ridge_is_finite(ridge):
                start = self._create_point_with_restored_coordinates(
                    vertices[ridge[0]][0], vertices[ridge[0]][1]
                )
                end = self._create_point_with_restored_coordinates(
                    vertices[ridge[1]][0], vertices[ridge[1]][1]
                )
                linestrings.append(LineString([start, end]))

        # 使用空间索引快速筛选内部线段
        tree = STRtree(linestrings)
        # 注意：query 返回的是索引，predicate="contains" 要求线段完全在内部
        indices = tree.query(self._input_geometry, predicate="contains")
        contained = [linestrings[i] for i in indices]

        if len(contained) < 2:
            raise TooFewRidgesError("Insufficient ridges to form a centerline")

        return unary_union(contained)

    def _get_voronoi_vertices_and_ridges(self) -> tuple:
        borders = self._get_densified_borders()
        voronoi = Voronoi(borders)
        return voronoi.vertices, voronoi.ridge_vertices

    @staticmethod
    def _ridge_is_finite(ridge: list) -> bool:
        return -1 not in ridge

    def _create_point_with_restored_coordinates(self, x: float, y: float) -> tuple:
        return (x + self._min_x, y + self._min_y)

    def _get_densified_borders(self) -> np.ndarray:
        points = []
        polygons = self._extract_polygons_from_input_geometry()
        for poly in polygons:
            # 外边界
            points += self._get_interpolated_boundary(poly.exterior)
            # 内环
            for interior in poly.interiors:
                points += self._get_interpolated_boundary(interior)
        # 若点数超出限制，随机采样（降采样）
        if len(points) > self._max_points:
            idx = np.random.choice(len(points), self._max_points, replace=False)
            points = [points[i] for i in idx]
        return np.array(points)

    def _extract_polygons_from_input_geometry(self):
        if isinstance(self._input_geometry, MultiPolygon):
            yield from self._input_geometry.geoms
        else:
            yield self._input_geometry

    def _get_interpolated_boundary(self, boundary) -> list:
        """将边界线等距插值，返回点列表（坐标已平移）"""
        line = LineString(boundary)
        first = self._get_coordinates_of_first_point(line)
        last = self._get_coordinates_of_last_point(line)
        inter = self._get_coordinates_of_interpolated_points(line)
        return [first] + inter + [last]

    def _get_coordinates_of_first_point(self, line: LineString) -> tuple:
        return self._create_point_with_reduced_coordinates(line.xy[0][0], line.xy[1][0])

    def _get_coordinates_of_last_point(self, line: LineString) -> tuple:
        return self._create_point_with_reduced_coordinates(line.xy[0][-1], line.xy[1][-1])

    def _get_coordinates_of_interpolated_points(self, line: LineString) -> list:
        pts = []
        d = self._interpolation_distance
        L = line.length
        # 防止死循环：若插值距离小于 1e-9，直接返回空
        if d < 1e-9:
            return pts
        while d < L:
            pt = line.interpolate(d)
            pts.append(self._create_point_with_reduced_coordinates(pt.x, pt.y))
            d += self._interpolation_distance
        return pts

    def _create_point_with_reduced_coordinates(self, x: float, y: float) -> tuple:
        return (x - self._min_x, y - self._min_y)


def multi_polygon_centerlines(
    polygons: List[Union[Polygon, MultiPolygon]],
    interpolation_distance: Optional[float] = None,
    max_points: int = 10000,
    parallel: bool = False,
) -> MultiLineString:
    """
    对多个多边形（或多部分多边形）分别生成中心线，并合并为一条多段线集合。

    :param polygons: 多边形或复合多边形列表
    :param interpolation_distance: 插值间距（None 表示自适应）
    :param max_points: 每个多边形的最大边界点数
    :param parallel: 是否并行处理
    :return: 合并后的中心线（MultiLineString）
    """
    all_lines = []

    def process_one(geom):
        try:
            cl = Centerline(geom, interpolation_distance, max_points)
            return cl.geometry
        except TooFewRidgesError:
            return None

    if parallel:
        with concurrent.futures.ProcessPoolExecutor() as executor:
            results = executor.map(process_one, polygons)
        for res in results:
            if res is not None:
                all_lines.append(res)
    else:
        for index,geom in enumerate(polygons):
            res = process_one(geom)
            if res is not None:
                all_lines.append(res)

    if not all_lines:
        return MultiLineString()
    return unary_union(all_lines)


def plot_polygons_and_centerlines(
    polygons: List[Polygon], centerlines: MultiLineString, show_holes=True
):
    """
    可视化多边形和中心线。

    :param polygons: 多边形列表
    :param centerlines: 中心线（MultiLineString）
    :param show_holes: 是否绘制内环（默认绘制）
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    for poly in polygons:
        # 外环
        x, y = poly.exterior.xy
        ax.fill(x, y, alpha=0.3, fc="lightblue", ec="blue", linewidth=0.5)

        # 内环
        if show_holes:
            for interior in poly.interiors:
                x_i, y_i = interior.xy
                ax.plot(x_i, y_i, color="blue", linewidth=0.5, linestyle="--")

    # 中心线
    if not centerlines.is_empty:
        for line in centerlines.geoms:
            x, y = line.xy
            ax.plot(x, y, color="red", linewidth=0.8)

    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Polygons and their Centerlines")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linestyle="--", alpha=0.7)
    # plt.show()
    # save
    plt.savefig("result_plot/polygons_and_centerlines.png", dpi=800)


def save_centerlines_to_geojson(centerlines: MultiLineString, filename: str):
    """
    将中心线保存为 GeoJSON 文件。

    :param centerlines: 中心线（MultiLineString）
    :param filename: 输出文件名
    """
    features = []
    for line in centerlines.geoms:
        coords = list(line.coords)
        features.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": coords,
                },
                "properties": {},
            }
        )
    geojson = {"type": "FeatureCollection", "features": features}
    with open(filename, "w") as f:
        json.dump(geojson, f, indent=2)


def main():

    # 读取json数据
    with open("data/polygon_output.json", "r") as f:
        data = json.load(f)



    outter_polygons = []
    inner_polygons = []
    for feature in data["features"]:
        coords = feature["geometry"]["coordinates"]
        coords = [tuple(coord) for coord in coords[0]] 
        
        line = LineString(coords)  # 这里假设外环在第一个坐标列表中
        simplified = line.simplify(0.01, preserve_topology=True)
        new_coords = list(simplified.coords)  # 闭合多边形
        
        is_outer = feature["properties"]["is_outer"]
        if is_outer:
            # 外环
            outter_polygons.append(new_coords)
        else:
            # 内环
            inner_polygons.append(new_coords)


    # 将所有的内环都绑定到第一个外环上
    if outter_polygons:
        main_polygon = Polygon(outter_polygons[0], holes=inner_polygons)
        polygons = [main_polygon]
    else:
        print("没有外环数据，无法构建多边形。")
        return

    # 生成中心线
    centerlines = multi_polygon_centerlines(polygons, interpolation_distance=0.8)

    print("中心线数量:", len(centerlines.geoms) if not centerlines.is_empty else 0)
    if not centerlines.is_empty:
        for i, line in enumerate(centerlines.geoms):
            print(f"  Line {i+1}: {line.wkt[:100]}...")  # 截断显示

    # 可视化
    plot_polygons_and_centerlines(polygons, centerlines)

    # save
    save_centerlines_to_geojson(centerlines, "data/centerlines_output.geojson")


if __name__ == "__main__":
    main()