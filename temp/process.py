from Proj.process.intersection_generate import *
from Proj.geometry.geometry_collect import *
from Proj.process.park_tope import *
from Proj.common.stat_related import *
import json,pymap3d
from typing import Tuple
import collections
from shapely import LineString, Polygon
import os


class GenerateFloorMap:
    """生成每一层的地图数据"""
    def __init__(
        self,
        floorID = None,
        floor_num = 20,
        max_search_radius:float = 3.0
    ) -> None:

        self.max_search_radius = max_search_radius

        self.lane_input = list()            #车道中心线
        self.slot_input = list()            #停车位
        self.laneline_input = list()        #车道边界线
        self.pillow_input = list()          #柱子
        self.entrance_input = list()        #出入口点
        self.cross_floor_input = list()     #跨层点
        self.landmark_input = list()        #地面标识

        self.ref_gps = None                 #参考点经纬度
        self.floorID = floorID              #停车楼层ID
        self.floor_num = floor_num          #停车楼层两位随机编号

        # 保存相关要素
        self.junction_list = list()         #清洗后的路口面数据
        self.link_list = list()             #清洗后的link数据
        self.linknode_list = list()         #清洗后的node点数据
        self.laneline_list = list()         #清洗后的车道边界线数据
        self.slot_list = list()             #清洗后的停车位数据
        self.landmark_list = list()         #清洗后的地面标识数据
        self.poi_list = list()              #清洗后的兴趣点数据



    def __call__(self, geojson_file_path,output_dir):
        """
        __call__ 的 Docstring
        :param geojson_file_path: 输入图纸的数据位置
        """

        # step1 加载数据
        if not self.init_dataset(geojson_file_path):
            return 

        # step2 中心线处理
        finder = PolylineIntersectionFinder( self.lane_input)
        intersections = finder.find_intersections()
        result = finder.split_and_renumber_polylines()
        
        # step3 创建buffer
        cross_buffer = CrossBuffer(intersection_result=result)
        buffer_segments = cross_buffer.generate_buffer_segments(debug=False)
        merged_buffers = cross_buffer.generate_and_merge_buffers(debug=False)
        
        junctions = dict()
        for key ,polygeo in merged_buffers.items():
            junctions[key] = list(polygeo.exterior.coords)

        self.generate_link_node_junction(result,junctions)
        self.generate_laneline(junctions)
        self.generate_landmark()
        self.generate_slot()


        self.result_output(output_dir)

    

    def init_dataset(self,file_path):
        # 加载geojson
        with open(file_path, 'r') as f:
            geojson_data = json.load(f)

        # 加载中心线、停车位、车位边线、柱子
        lane_input_lonlat = list()
        laneline_input_lonlat= list()
        slot_input_lonlat= list()
        pillow_input_lonlat= list()
        entrance_point_input= list()
        crossfloor_point_input = list()


        all_coords = list()
        for feature in geojson_data['features']:

            if feature['geometry'] is None or feature['properties']is None:
                continue

            feature_name = feature['properties']['entity_name']
            feature_geometry  = feature['geometry']['coordinates']
            
            if  feature_name== '车道中心线':
                lane_input_lonlat.append(feature_geometry)

            elif feature_name == '车道边界线':
               laneline_input_lonlat.append(feature_geometry)

            elif feature_name == '停车位':
                slot_input_lonlat.append(feature_geometry[0])# 车位是Polygon，取第一个环

            elif feature_name== "柱":
                pillow_input_lonlat.append(feature_geometry)# 柱是Polygon，取第一个环

            elif feature_name == "出入口":
                entrance_point_input.append(feature_geometry)# 出入口是Point

            elif feature_name == "跨层点":
                crossfloor_point_input.append(feature_geometry)# 跨层点是Point

            else:
                print(f"unknown type {feature_name}")

            if str(feature['geometry']["type"]).upper() == "LINESTRING":
                all_coords.extend(feature_geometry)
            elif str(feature['geometry']["type"]).upper() == "POINT":
                all_coords.append(feature_geometry)
            elif str(feature['geometry']["type"]).upper() == "POLYGON":
                all_coords.extend(feature_geometry[0])


        # 找到经纬度最小点作为参考点
        
        lons = [c[0] for c in all_coords]
        lats = [c[1] for c in all_coords]

        ref_lon, ref_lat = min(lons),min(lats)
        self.ref_gps = [ref_lon,ref_lat]
        # 以第一个点为基准，转换成enu坐标系
        
        
        # 中心线数据必必须要有
        if not lane_input_lonlat:
            return False
        
        
        # 转换到enu
        for i in range(len(lane_input_lonlat)):

            trans_result = [self.lonlat_to_enu(lon, lat, ref_lon, ref_lat) for lon, lat in lane_input_lonlat[i]]
            self.lane_input.append(trans_result)


        for i in range(len(laneline_input_lonlat)): 
            trans_result = [self.lonlat_to_enu(lon, lat, ref_lon, ref_lat) for lon, lat in laneline_input_lonlat[i]]
            self.laneline_input.append(trans_result)


        for i in range(len(slot_input_lonlat)): 
            trans_result = [self.lonlat_to_enu(lon, lat, ref_lon, ref_lat) for lon, lat in slot_input_lonlat[i]]
            self.slot_input.append(trans_result)


        for i in range(len(pillow_input_lonlat)): 
            trans_result = [self.lonlat_to_enu(lon, lat, ref_lon, ref_lat) for lon, lat in pillow_input_lonlat[i]]
            self.pillow_input.append(trans_result)


        for i in range(len(entrance_point_input)): 
            (lon, lat) = entrance_point_input[i]
            trans_result = self.lonlat_to_enu(lon, lat, ref_lon, ref_lat) 
            self.entrance_input.append(trans_result)


        for i in range(len(crossfloor_point_input)): 
            (lon, lat) = crossfloor_point_input[i]
            trans_result = self.lonlat_to_enu(lon, lat, ref_lon, ref_lat) 
            self.cross_floor_input.append(trans_result)

        return True

    def generate_link_node_junction(self,input_data:IntersectionResult,junctions:dict,bidirect:bool = True):
        """
        generate_link_node_junction 的 Docstring
        
        :param self: 说明
        :param input_data: 交叉点以及车道中心线
        :type input_data: IntersectionResult
        :param junctions: 交叉点坐标对应的交叉路口矢量
        :type junctions: dict
        """


        input_data.intersection_to_renumbered_ids
        all_polylines = input_data.renumbered_polylines
        # intersections = input_data.intersections

        # 利用中心线，构建node和link
        nodecoord_related_linknode = dict()
        startnodecoord_related_links =collections.defaultdict(list) #node为起点的时候，关联的link
        endnodecoord_related_links =collections.defaultdict(list) #node为终点的时候，关联的link

        for line_id, coords in all_polylines: 
            start_point = (coords[0][0],coords[0][1])
            end_point = (coords[-1][0],coords[-1][1])  

            if start_point not in nodecoord_related_linknode:
                start_node = LinkNode()
                start_node.set_node_id(self.floor_num)
                start_node.set_point(start_point)
                nodecoord_related_linknode[start_point] = start_node
            else:
                start_node = nodecoord_related_linknode[start_point]

            if end_point not in nodecoord_related_linknode:
                end_node = LinkNode()
                end_node.set_node_id(self.floor_num)
                end_node.set_point(end_point)    
                nodecoord_related_linknode[end_point] = end_node
            else:
                end_node = nodecoord_related_linknode[end_point]

            # 两个方向的两条link
            link1 = Link()
            link1.set_centerline(coords)
            link1.set_link_id(self.floor_num)
            link1.start_node = start_node.LinkNodeID
            link1.end_node = end_node.LinkNodeID

            link2 = Link()
            link2.set_centerline(coords[::-1])
            link2.set_link_id(self.floor_num)
            link2.start_node = end_node.LinkNodeID
            link2.end_node = start_node.LinkNodeID
            
            # 保存link
            self.link_list.append(link1)
            self.link_list.append(link2)

            # 记录每个node作为起点和终点时关联的link
            startnodecoord_related_links[start_node.LinkNodeID].append(link1)
            endnodecoord_related_links[start_node.LinkNodeID].append(link2)

            startnodecoord_related_links[end_node.LinkNodeID].append(link2)
            endnodecoord_related_links[end_node.LinkNodeID].append(link1)

        # 所有的linknode
        self.linknode_list =list(nodecoord_related_linknode.values())
        # 根据node，关联相关的junction
        for intersec_pt ,polygon in junctions.items():

            intersect_node = nodecoord_related_linknode[intersec_pt]
            junction = LinkJunction()
            junction.set_junction_id(self.floor_num)
            junction.ExitLinkIDs = [link.LinkID for link in startnodecoord_related_links[intersect_node.LinkNodeID]]
            junction.EnterLinkIDs = [link.LinkID for link in endnodecoord_related_links[intersect_node.LinkNodeID]]
            junction.set_junction_polygon(polygon)

            self.junction_list.append(junction)

    def generate_laneline(self,junctions):
        """构建车道边界线,车道线与车道面求非"""
        
        geoline_list = list()
        for line in self.laneline_input:
            polyline = LineString(line)
            geoline_list.append(polyline)
            
        geopoly_list = list()
        for intersecpt, polygon in junctions.items():
            poly = Polygon(polygon)
            geopoly_list.append(poly)

        clipper = PolylineClipper(geopoly_list)
        clipped_polylines = clipper.clip_batch(geoline_list)

        final_lines = list()
        for i, clippolyline in enumerate(clipped_polylines, 1):
            if clippolyline.geom_type == "MultiLineString":
                for j, seg in enumerate(clippolyline.geoms, 1):
                    final_lines.append(list(seg.coords))
            else:
                final_lines.append(list(clippolyline.coords))
                
        for line in final_lines:
            line_pline = LaneLine(FloorID= self.floorID,Color=LaneLineColor.unknown)
            line_pline.set_laneline_id(self.floor_num)
            line_pline.set_laneline(line)
            self.laneline_list.append(line_pline)
        
    def generate_landmark(self,):
        """构建地面标识"""
        for polygon in self.landmark_input:
            ldmk = LandMark()
            ldmk.set_landmark_id(self.floor_num)
            ldmk.set_polygon(polygon)

            self.landmark_list.append(ldmk)
        
    def generate_slot(self,):
        """生成停车位"""
        for slotcoords in self.slot_input:
            slotpoly = Polygon(slotcoords)
            nearest_link_id  = self.search_nearest_link(slotpoly)

            slot = Slot()
            slot.set_slot_id(self.floor_num)
            slot.LinkID=nearest_link_id
            slot.set_polygon(slotcoords)
            self.slot_list.append(slot)


    def result_output(self,outdir:str = "result_out"):
        """保存结果"""
        os.makedirs(outdir,exist_ok=True)

        linknode_path = os.path.join(outdir,"LinkNode.geojson")
        geojson_node = self.get_geojson_header()
        for node in self.linknode_list:
            feature = node.to_geojson_feature()
            geojson_node["features"].append(feature)
        with open(linknode_path,'w')as fp:
            json.dump(geojson_node,fp,indent=2)


        link_path = os.path.join(outdir,"Link.geojson")
        geojson_link = self.get_geojson_header()
        for link in self.link_list:
            feature = link.to_geojson_feature()
            geojson_link["features"].append(feature)
        with open(link_path,'w')as fp:
            json.dump(geojson_link,fp,indent=2)


        junction_path = os.path.join(outdir,"LinkJunction.geojson")
        geojson_junction = self.get_geojson_header()
        for junc in self.junction_list:
            feature = junc.to_geojson_feature()
            geojson_junction["features"].append(feature)
        with open(junction_path,'w')as fp:
            json.dump(geojson_junction,fp,indent=2)


        line_path = os.path.join(outdir,"LaneLine.geojson")
        geojson_line = self.get_geojson_header()
        for line in self.laneline_list:
            feature = line.to_geojson_feature()
            geojson_line["features"].append(feature)
        with open(line_path,'w')as fp:
            json.dump(geojson_line,fp,indent=2)


        slot_path = os.path.join(outdir,"Slot.geojson")
        geojson_slot = self.get_geojson_header()
        for slot in self.slot_list:
            feature = slot.to_geojson_feature()
            geojson_slot["features"].append(feature)
        with open(slot_path,'w')as fp:
            json.dump(geojson_slot,fp,indent=2)


        ldmk_path = os.path.join(outdir,"LandMark.geojson")
        geojson_ldmk = self.get_geojson_header()
        for ldmk in self.landmark_list:
            feature = ldmk.to_geojson_feature()
            geojson_ldmk["features"].append(feature)
        with open(ldmk_path,'w')as fp:
            json.dump(geojson_ldmk,fp,indent=2)


        poi_path = os.path.join(outdir,"POI.geojson")
        geojson_poi = self.get_geojson_header()
        for poi in self.poi_list:
            feature = poi.to_geojson_feature()
            geojson_poi["features"].append(feature)
        with open(poi_path,'w')as fp:
            json.dump(geojson_poi,fp,indent=2)







# ##################基本函数##########################

    def lonlat_to_enu(self, lon: float, lat: float, ref_lon: float, ref_lat: float) -> Tuple[float, float]:
        """使用pymap3d将经纬度转换为ENU坐标系"""
        x, y, _ = pymap3d.geodetic2enu(lat, lon, 0, ref_lat, ref_lon, 0)
        return (x, y)

    def search_nearest_link(self,parklot:Polygon):
        """在指定半径内搜索最近的车道"""
        nearest_link = None
        min_distance = float('inf')
        for link in self.link_list:
            polyline = LineString(link.LinkLineString)
            distance = parklot.distance(polyline)
            if distance < min_distance and distance <= self.max_search_radius:
                min_distance = distance
                nearest_link = link
        if nearest_link is None:
            return None
        
        return nearest_link.LinkID

    def get_geojson_header(self,):
        """返回geojson头"""
        header = {
            "type": "FeatureCollection",
            "name": self.floorID,
            "features" : []
        }
        return header
