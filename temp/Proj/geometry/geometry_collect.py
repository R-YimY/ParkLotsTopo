""" Created at:2026/03/30 17:15:27,@Author: yimy.
构建要素类
"""

from Proj.common.stat_related import *
from shapely.geometry import LineString, Polygon
import pymap3d,random

__all__ = ["ParkingInfo","Floor",
           "LinkNode","Link","LinkJunction",
           "LaneLine","Slot","LandMark","POI"]

def random_n_digit_str(N):
    # 生成 N 位随机数字字符串，允许首位为0
    return ''.join(str(random.randint(0, 9)) for _ in range(N))

def fix_polygon_order(vertices,):
    """
    判断多边形顶点的环绕方向，逆时针则反转列表
    :param vertices: 多边形顶点列表，格式为 [(x0,y0), (x1,y1), ..., (xn,yn)]
    :return: 处理后的顶点列表（保证顺时针）
    """


    n = len(vertices)
    # 边界校验：多边形至少需要3个顶点
    if n < 3:
        raise ValueError("多边形顶点数量不能小于3")
    

    # 计算有向面积的求和项（仅符号有用，无需除以2）
    sum_area = 0
    for i in range(n):
        vertices[i] = list(vertices[i])
        x_i, y_i = vertices[i]
        # 下一个顶点：最后一个顶点的下一个是第一个
        x_j, y_j = vertices[(i + 1) % n]
        sum_area += (x_i * y_j) - (x_j * y_i)
    

    if sum_area > 0:
        # 逆时针，需要反转
        vertices = vertices[::-1]  # 反转列表


    # 计算经纬度最大最小坐标值
    xs = [x for x, y in vertices]
    ys = [y for x, y in vertices]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    box_coors = [
        [min_x, min_y],
        [max_x, max_y]
    ]

    return vertices, box_coors


class ParkingInfo:
    """停车场信息相关属性，一个停车场"""
    def __init__(
        self,
        province:str = "chongqing",
        map_type:str = "CadMap",
        floors: int = 2,
        slots_num: int = 0,
        Hlimit: int = 0,
        district: str = "liangjiang",
        adddress: str = "liangjiang",
        park_type = "indoor",

    ):
        self.province = province
        self.map_type = map_type
        self.floors = floors
        self.slots_num = slots_num
        self.Hlimit = Hlimit
        self.district = district
        self.adddress = adddress
        self.park_type = park_type


        self.ParkingID = None
        self.BoundingBox  = None
        self.vertices_lonlat = None


    def set_park_id(self,num_digits=6):
        """生成停车场ID，格式为：省份代码+地图类型代码+随机数"""
        province_code = ProvinceCode.__dict__.get(self.province, None)
        mapstat_code = MapCode.__dict__.get(self.map_type, None)
        median_random = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        ParkingIDstr =  f"{ParkInfosCode.ParkCode}{province_code}{median_random}{mapstat_code}"
        self.ParkingID = int(ParkingIDstr)

    def set_polygon(self,polygon):
        """设置停车场的几何信息"""
        self.vertices_lonlat, self.BoundingBox = fix_polygon_order(polygon)

    def to_geojson_feature(self):
        if self.ParkingID is None:
            self.set_park_id()

        parkinfo_geojson_feature =                 {
                    "type": "Feature",
                    "properties": {
                        "province_code": ProvinceCode.__dict__.get(self.province, None),
                        "map_code": MapCode.__dict__.get(self.map_type, None),
                        "floors": self.floors,
                        "slots_num": self.slots_num,
                        "Hlimit": self.Hlimit,
                        "district": self.district,
                        "address": self.adddress,
                        "park_type": ParkType.__dict__.get(self.park_type, None),
                        "BoundingBox": self.BoundingBox
                    },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [self.vertices_lonlat]
                    }
                }
        return parkinfo_geojson_feature


class Floor:
    """楼层信息相关属性，一层"""
    def __init__(
        self,
        FloorID = None,
        FloorLevel:int = -1,
        FloorName:str = "B1",
        ParkingID= None,
    ):
        self.FloorID = FloorID
        self.FloorLevel = FloorLevel
        self.FloorName = FloorName
        self.ParkingID = ParkingID

        self.vertices_lonlat = None
        self.BoundingBox = None


    def set_floor_id(self,num_digits=2):
        """生成楼层ID，格式为：停车场ID+楼层编号"""
        code_random = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        FloorIDstr = f"{ParkInfosCode.FloorCode}{code_random}"
        self.FloorID = int(FloorIDstr)



    def set_polygon(self,polygon):
        """设置楼层的几何信息"""
        self.vertices_lonlat, self.BoundingBox = fix_polygon_order(polygon)
    
    def to_geojson_feature(self):
        floorinfo_geojson_feature =  {
               
                    "type": "Feature",
                    "properties": {
                        "FloorID": self.FloorID,
                        "FloorLevel": self.FloorLevel,
                        "FloorName": self.FloorName,
                        "ParkingID": self.ParkingID,

                    },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [self.vertices_lonlat]
                    }
                }

        return floorinfo_geojson_feature


class LinkNode:
    """路口节点信息相关属性，一个顶点"""
    def __init__(
        self,
        LinkNodeID = None,
        AssociatedPOIIDs = list(),
        AssociatedJunctionID = None,
        FloorID = None,
    ):
        self.LinkNodeID = LinkNodeID
        self.AssociatedPOIIDs = AssociatedPOIIDs
        self.AssociatedJunctionID = AssociatedJunctionID
        self.FloorID = FloorID

        self.point = None

        # 一些挂接相关的信息
        self.related_link = list()
        self.related_junction  = list()


    def set_node_id(self,floornumber:int = 2,num_digits=5):
        """生成路口节点ID，格式为：floorid+index"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        floornumberstr = f"{ParkInfosCode.LinkNodeCode}{floornumber}{randomnumber}"
        self.LinkNodeID = int(floornumberstr)

    def set_point(self,point):
        """设置路口节点的几何信息"""
        self.point = [point[0],point[1]]

    def to_geojson_feature(self):
        if self.LinkNodeID is None:
            self.set_node_id()

        feature = {
            "type": "Feature",
            "properties": {
                "LinkNodeID": self.LinkNodeID,
                "FloorID": self.FloorID,
                "AssociatedPOIIDs": self.AssociatedPOIIDs,
                "AssociatedJunctionID": self.AssociatedJunctionID
            },
            "geometry": {
                "type": "Point",
                "coordinates": self.point
            }
        }
        return feature


class Link:
    """车道中心线信息相关属性,一条中心线"""
    def __init__(
        self,
        LinkID = None,
        FrontLinkIDs = list(),
        SuccessorLinkIDs = list(),
        FloorID = None,
        BidirectionalRoadIDs = list(),
        LinkType = None,
        LaneLineIDs = list(),
        two_sided = True, #是否双向
    ):
        self.LinkID = LinkID
        self.FrontLinkIDs = FrontLinkIDs
        self.SuccessorLinkIDs = SuccessorLinkIDs
        self.FloorID = FloorID
        self.BidirectionalRoadIDs = BidirectionalRoadIDs    
        self.LinkType = LinkType
        self.LaneLineIDs = LaneLineIDs
        
        self.LinkLength = None
        self.StartLinkNodeID = None
        self.EndLinkNodeID = None

        self.two_sided = two_sided

        # 保存几何信息
        self.LinkLineString = None
        self.start_node = None
        self.end_node = None


    def set_link_id(self, floornum:int =2, num_digits=5):
        """生成车道ID"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        LinkIDstr = f"{ParkInfosCode.LinkCode}{floornum}{randomnumber}"
        self.LinkID = int(LinkIDstr)

    def set_centerline(self,centerline):
        """设置车道中心线的几何信息"""

        polyline = LineString(centerline)
        self.LinkLineString = centerline
        self.LinkLength = polyline.length



    def set_link_type(self,linktype = LinkConnectionType.inner_plant):
        """设置车道类型"""
        self.LinkType = linktype



    def to_geojson_feature(self):
        if self.LinkID is None:
            self.set_link_id()

        link_info_geojson_feature = {
            "type": "Feature",
            "properties": {
                "LinkID": self.LinkID,
                "FrontLinkIDs": self.FrontLinkIDs,
                "SuccessorLinkIDs": self.SuccessorLinkIDs,
                "FloorID": self.FloorID,
                "BidirectionalRoadIDs": self.BidirectionalRoadIDs,
                "LinkType": self.LinkType,
                "LaneLineIDs": self.LaneLineIDs,
                "LinkLength": self.LinkLength,
                "StartLinkNodeID": self.StartLinkNodeID,
                "EndLinkNodeID": self.EndLinkNodeID,
                "two_sided": self.two_sided
            },
            "geometry": {
                "type": "LineString",
                "coordinates": list(self.LinkLineString)
            }
        }
        return link_info_geojson_feature


class LinkJunction:
    """车道交叉口信息相关属性,一个交叉口"""
    def __init__(
        self,
        JunctionID = None,          #道路路口面编号
        EnterLinkIDs = list(),      #进入路口的车道ID列表
        ExitLinkIDs = list(),       #离开路口的车道ID列表
        FloorID = None,             #所属楼层ID
    ):
        self.JunctionID = JunctionID
        self.EnterLinkIDs = EnterLinkIDs
        self.ExitLinkIDs = ExitLinkIDs
        self.FloorID = FloorID

        self.points = None
    
    
    def set_junction_id(self, floornum:int =2, num_digits=5):
        """生成路口面ID"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        JunctionIDstr = f"{ParkInfosCode.JunctionCode}{floornum}{randomnumber}"
        self.JunctionID = int(JunctionIDstr)

    def set_junction_polygon(self,polygon):
        """设置路口的几何信息"""
        self.points, _ = fix_polygon_order(polygon)

    def to_geojson_feature(self):
        if self.JunctionID is None:
            self.set_junction_id()

        junction_info_geojson_feature = {
            "type": "Feature",
            "properties": {
                "JunctionID": self.JunctionID,
                "EnterLinkIDs": self.EnterLinkIDs,
                "ExitLinkIDs": self.ExitLinkIDs,
                "FloorID": self.FloorID
            },
            "geometry": {
                "type": "Polygon",
                "coordinates": [self.points]
            }
        }
        return junction_info_geojson_feature


class LaneLine:
    def __init__(
        self,
        LaneLineID = None,
        Color = None,
        LineType = None,
        FloorID = None,
    ):
        self.LaneLineID = LaneLineID
        self.Color = Color
        self.FloorID = FloorID
        self.LineType = LineType

        self.LaneLineString = None

    def set_laneline_id(self, floornum:int =2, num_digits=5):
        """生成车道线ID"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        LaneLineIDstr = f"{ParkInfosCode.LaneLineCode}{floornum}{randomnumber}"
        self.LaneLineID = int(LaneLineIDstr)

    def set_laneline(self,laneline):
        """设置车道线的几何信息"""

        polyline = LineString(laneline)
        self.LaneLineString = polyline

    def to_geojson_feature(self):
        if self.LaneLineID is None:
            self.set_laneline_id()

        laneline_info_geojson_feature = {
            "type": "Feature",
            "properties": {
                "LaneLineID": self.LaneLineID,
                "Color": self.Color,
                "LineType": self.LineType,
                "FloorID": self.FloorID
            },
            "geometry": {
                "type": "LineString",
                "coordinates": list(self.LaneLineString.coords)
            }
        }
        return laneline_info_geojson_feature


class Slot:
    """停车位信息相关属性,一个车位"""
    def __init__(
        self,
        SlotID = None,
        SlotType = None,
        FloorID = None,
        Code = None,  #库位号
        ChargingPile = None,
        Status = None,  #车位状态，如：空闲、占用、维修等
        StatusUpdateTime = None,
        Lock = None,  #车位锁
        Limiter = None,  #车位限位器
        LinkID = None,  #车位所属车道ID
    ):
        self.SlotID = SlotID
        self.SlotType = SlotType
        self.FloorID = FloorID
        self.Code = Code
        self.ChargingPile = ChargingPile
        self.Status = Status
        self.StatusUpdateTime = StatusUpdateTime
        self.Lock = Lock
        self.Limiter = Limiter
        self.LinkID = LinkID

        self.points = None
        self.BoundingBox = None

    def set_slot_id(self, floornum:int =2, num_digits=5):
        """生成车位ID"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        SlotIDstr = f"{ParkInfosCode.SlotCode}{floornum}{randomnumber}"
        self.SlotID = int(SlotIDstr)

    def set_polygon(self,polygon):
        """设置车位的几何信息"""
        self.points, self.BoundingBox = fix_polygon_order(polygon)

    def to_geojson_feature(self):
        if self.SlotID is None:
            self.set_slot_id()

        slot_info_geojson_feature = {
            "type": "Feature",
            "properties": {
                "SlotID": self.SlotID,
                "SlotType": self.SlotType,
                "FloorID": self.FloorID,
                "Code": self.Code,
                "ChargingPile": self.ChargingPile,
                "Status": self.Status,
                "StatusUpdateTime": self.StatusUpdateTime,
                "Lock": self.Lock,
                "Limiter": self.Limiter,
                "LinkID": self.LinkID
            },
            "geometry": {
                "type": "Polygon",
                "coordinates": [self.points]
            }
        }
        return slot_info_geojson_feature
    

class LandMark:
    """路面标志"""
    def __init__(
        self,
        LandMarkID = None,
        Type = None,
        LinkID = None,
    ):
        self.LandMarkID = LandMarkID
        self.Type = Type
        self.LinkID = LinkID

        self.vertices_lonlat = None
        self.BoundingBox = None

    def set_landmark_id(self, floornum:int =2, num_digits=5):
        """生成路面标志ID"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        LandMarkIDstr = f"{ParkInfosCode.LandMarkCode}{floornum}{randomnumber}"
        self.LandMarkID = int(LandMarkIDstr)
    
    def set_polygon(self,polygon):
        """设置路面标志的几何信息"""
        self.vertices_lonlat, self.BoundingBox = fix_polygon_order(polygon)

    def to_geojson_feature(self):

        if self.LandMarkID is None:
            self.set_landmark_id()

        landmark_info_geojson_feature = {
            "type": "Feature",
            "properties": {
                "LandMarkID": self.LandMarkID,
                "Type": self.Type,
                "LinkID": self.LinkID
            },
            "geometry": {
                "type": "Polygon",
                "coordinates": [self.vertices_lonlat]
            }
        }
        return landmark_info_geojson_feature
    
    
class POI:
    """兴趣点信息相关属性,一个兴趣点"""
    def __init__(
    self,
        POIID = None,
        POIType = None,
        FloorID = None,
        POIName = None,
        ConnectedPOIIDs = list(),  #与该POI相关联的其他停车场POI的ID列表
    ):
        self.POIID = POIID
        self.POIType = POIType
        self.FloorID = FloorID
        self.POIName = POIName
        self.ConnectedPOIIDs = ConnectedPOIIDs


        self.point = None

    def set_poi_id(self, floornum:int =2, num_digits=5):
        """生成兴趣点ID"""
        randomnumber = random_n_digit_str(num_digits)  # 生成一个随机数作为ID的一部分
        POIIDstr = f"{ParkInfosCode.POICode}{floornum}{randomnumber}"
        self.POIID = int(POIIDstr)
    
    def set_point(self,point):
        """设置兴趣点的几何信息"""
        self.point = point
    
    def to_geojson_feature(self):
        if self.POIID is None:
            self.set_poi_id()

        poi_info_geojson_feature = {
            "type": "Feature",
            "properties": {
                "POIID": self.POIID,
                "POIType": self.POIType,
                "FloorID": self.FloorID,
                "POIName": self.POIName,
                "ConnectedPOIIDs": self.ConnectedPOIIDs
            },
            "geometry": {
                "type": "Point",
                "coordinates": self.point
            }
        }
        return poi_info_geojson_feature