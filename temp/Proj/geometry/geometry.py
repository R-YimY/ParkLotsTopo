from shapely import Point,LineString,Polygon
import collections

class point_node:
    def __init__(self) -> None:
        self.id = None
        self.point_coord = None
        self.related_link = list()
        self.related_junction  = list()
    
class link_line:
    def __init__(self) -> None:
        self.id = None
        self.line_coords = None
        self.related_node = list()
        self.related_junction  = list()
        self.BidirectionalLinkID = None
        self.StartLinkNodeID = None
        self.EndLinkNodeID = None

        self.related_link = list()


class juncion_polygon:
    def __init__(self) -> None:
        self.id = None
        self.polygon_coords = None
        self.related_node = list()
        self.related_linkline  = list()
