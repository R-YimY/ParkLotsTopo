""" Created at:2026/03/23 16:47:24,@Author: yimy."""

import math
from typing import List, Tuple, Optional
from shapely.geometry import Polygon

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    
    def distance_to(self, other: 'Point') -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)
    
    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)
    
    def __repr__(self):
        return f"({self.x:.1f}, {self.y:.1f})"




class Polyline:
    def __init__(self, points: List[Point], closed: bool = False, idx: int = None):
        self.points = points
        self.closed = closed
        self.idx = idx

    

    @property
    def start(self) -> Point:
        return self.points[0] if self.points else None 
    
    @property
    def end(self) -> Point:
        return self.points[-1] if self.points else None
    
    def get_direction_at_start(self) -> Optional[Tuple[float, float]]:
        if len(self.points) < 2:
            return None
        p0, p1 = self.points[0], self.points[1]
        return (p1.x - p0.x, p1.y - p0.y)
    
    def get_length(self)-> float:
        if len(self.points) < 2:
            return 0.0
        
        length = 0
        for i in range(len(self.points) - 1):
            x1, y1 = self.points[i].x,  self.points[i].y
            x2, y2 = self.points[i + 1].x,self.points[i + 1].y
            length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return length



    def get_direction_at_end(self) -> Optional[Tuple[float, float]]:
        if len(self.points) < 2:
            return None
        p0, p1 = self.points[-2], self.points[-1]
        return (p1.x - p0.x, p1.y - p0.y)
    
    def reverse(self) -> 'Polyline':
        
        return Polyline(list(reversed(self.points)), self.closed, self.idx)


    def to_geojson(self) -> dict:
        # 闭合的保存为Polygon，开放的保存为LineString
        if self.closed:
            # Polygon需要首尾闭合，将第一个点加到末尾
            coords = [[p.x, p.y] for p in self.points]
            if coords and (coords[0] != coords[-1]):
                coords.append(coords[0])
            return {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [coords]
                }
            }
        else:
            return {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[p.x, p.y] for p in self.points]
                }
            }
    

    def endvector_distance_with(self, other: 'Polyline') -> Optional[float]:
        """计算两条多段线末端方向向量之间的距离"""
        vec1 = self.get_direction_at_end()
        vec2 = other.get_direction_at_end()
        if vec1 is None or vec2 is None:
            return None
        return math.hypot(vec1[0] - vec2[0], vec1[1] - vec2[1])


    def calculate_area(self) -> float:
        """使用Shoelace公式计算多边形面积，适用于闭合的Polyline"""
        if not self.closed or len(self.points) < 3:
            return 0.0

        # 使用第三方库计算面积，非凸多边形
        vertices = [(p.x, p.y) for p in self.points]
        poly = Polygon(vertices)
        return poly.area



    def __repr__(self):
        return f"Polyline({self.idx}, pts={len(self.points)}, closed={self.closed})"

