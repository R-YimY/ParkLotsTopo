from typing import List, Tuple, Dict
from shapely import Point, LineString

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
        self.poly_idx = poly_idx  # 所属多段线的编号
        self.seg_idx = seg_idx    # 多段线内的线段索引编号

    def get_seg_geom(self):
        linecoords = [(self.start.x, self.start.y), (self.end.x, self.end.y)]
        return LineString(linecoords)

    def __repr__(self):
        return f"Segment2D(poly_idx={self.poly_idx}, seg_idx={self.seg_idx})"


class Polyline2D:
    def __init__(self, vertices: List[Tuple[float, float]], poly_idx: int):
        self.poly_idx = poly_idx                            #多段线编号
        self.points = [Point2D(x, y) for x, y in vertices]  #多段线上的点
        self.segments = self._split_to_segments()           #多段线上的线段
        self.start = self.points[0]                         #多段线起点
        self.end = self.points[-1]                          #多段线终点
        self.intersect_point = None                         #交点
        self.intersect_point_related_segment = None         #交点相关的线段
    
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
