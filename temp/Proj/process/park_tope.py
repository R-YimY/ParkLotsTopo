""" Created at:2026/03/30 11:34:03,@Author: yimy.
通过几何关系构建拓扑关系
"""


from typing import Any,List, Optional,Tuple
from shapely.geometry import LineString, Polygon, MultiPolygon,MultiLineString
import matplotlib.pyplot as plt
import numpy as np
from shapely.ops import unary_union
from shapely.strtree import STRtree
from typing import List, Union, Iterable



__all__ = ['PolylineClipper']


class PolylineClipper:
    def __init__(self, polygons: Iterable[Union[Polygon, MultiPolygon]]):
        """
        初始化裁剪器：预处理多边形（合并、建索引）
        
        Args:
            polygons: 输入的多个多边形（Polygon 或 MultiPolygon 列表）
        """
        # 1. 修复无效几何（如自交多边形）并过滤空几何
        self.polygons = []
        for p in polygons:
            if not p.is_empty:
                fixed_p = p.buffer(0) if not p.is_valid else p  # buffer(0) 修复简单自交
                if not fixed_p.is_empty:
                    self.polygons.append(fixed_p)
        
        # 2. 合并所有多边形（减少差集运算次数）
        self.polygons_union = unary_union(self.polygons) if self.polygons else Polygon()
        
        # 3. 建立空间索引（用于快速筛选候选多边形，保留以扩展复杂场景）
        self.spatial_index = STRtree(self.polygons) if self.polygons else None

    def clip_single(self, polyline: Union[LineString, MultiLineString]) -> Union[LineString, MultiLineString]:
        """
        裁剪单条多段线，提取其在多边形外部的部分
        
        Args:
            polyline: 输入多段线（LineString 或 MultiLineString）
        
        Returns:
            裁剪后的多段线（空几何表示完全在多边形内部）
        """
        # 快速修复无效多段线
        if not polyline.is_valid:
            polyline = polyline.buffer(0)
        if polyline.is_empty:
            return LineString()  # 空输入直接返回空

        # 快速预判 1：多段线与合并后的多边形完全不相交 → 直接保留原多段线
        if self.polygons_union.is_empty or polyline.disjoint(self.polygons_union):
            return polyline
        
        # 快速预判 2：多段线完全在合并后的多边形内部 → 返回空几何
        if polyline.within(self.polygons_union):
            return LineString()
        
        # 核心运算：计算多段线与合并后多边形的差集
        return polyline.difference(self.polygons_union)

    def clip_batch(self, polylines: Iterable[Union[LineString, MultiLineString]], 
                   skip_empty: bool = True) -> List[Union[LineString, MultiLineString]]:
        """
        批量裁剪多段线
        
        Args:
            polylines: 输入的多个多段线列表
            skip_empty: 是否跳过完全在多边形内部的空结果
        
        Returns:
            裁剪后的多段线列表
        """
        results = []
        for polyline in polylines:
            clipped = self.clip_single(polyline)
            if not skip_empty or not clipped.is_empty:
                results.append(clipped)
        return results