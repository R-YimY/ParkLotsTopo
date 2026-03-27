'''
Author: YimY 99624564+R-YimY@users.noreply.github.com
Date: 2026-03-27 10:43:14'''
import numpy as np
import json
import pymap3d
from loguru import logger


class CoorsTrans:
    def __init__(self) -> None:
        self.ref_gps = None
    def __call__(self, match_points,cad_features):
        origincoords,trans_targetcood = self.match_points_init(match_points)

        transparams = self.calculate_7params(origincoords,trans_targetcood)

        trans_enu = self.transfeatures_enu(cad_features,transparams)

        # 转换回经纬度
        trans_geometry = self.transfeatures_geometry(trans_enu)

        return trans_geometry


    def calculate_7params(self,points_A: np.ndarray, points_B: np.ndarray) -> np.ndarray:
        """基于公共点解算布尔莎七参数"""
        n = points_A.shape[0]
        if n < 3:
            raise ValueError("七参数解算至少需要3个公共点！")
        
        A = []
        L = []
        for i in range(n):
            xa, ya, za = points_A[i]
            xb, yb, zb = points_B[i]
            
            A.append([1, 0, 0, 0, za, -ya, xa])
            L.append(xb)
            A.append([0, 1, 0, -za, 0, xa, ya])
            L.append(yb)
            A.append([0, 0, 1, ya, -xa, 0, za])
            L.append(zb)
        
        A = np.array(A, dtype=np.float64)
        L = np.array(L, dtype=np.float64).reshape(-1, 1)
        params = np.linalg.lstsq(A, L, rcond=None)[0].flatten()
        return params

    def transform_point(self,XA: float, YA: float, ZA: float, params: np.ndarray) -> tuple:
        """单个点七参数转换"""
        dx, dy, dz, wx, wy, wz, k = params
        xb = dx + k * (XA - wz * YA + wy * ZA)
        yb = dy + k * (wz * XA + YA - wx * ZA)
        zb = dz + k * (-wy * XA + wx * YA + ZA)
        return round(xb, 9), round(yb, 9), round(zb, 9)

    def match_points_init(self,match_points,z_default: float = 0.0):
        """将经纬度转换成enu"""
        target_lonlat_pts =  match_points["target"]
        origin_pts = match_points["origin"]

        targetcoords  = list()
        for pt in target_lonlat_pts:
            targetcoords.append(pt['coors'])
        # 将第一个点作为转换的点
        self.ref_gps = targetcoords[0]

        # 转换成enu
        trans_targetcood = list()
        for index, targetcood in enumerate(targetcoords):

            e, n, u = pymap3d.geodetic2enu(targetcood[1], targetcood[0], z_default, self.ref_gps[1], self.ref_gps[0], z_default)
            trans_targetcood.append([e, n, u])
        
        # 获取geojson的坐标
        origincoords  = list()
        for pt in origin_pts:
            origincoords.append([pt['coors'][0],pt['coors'][1],z_default])

        return np.asarray(origincoords),np.asarray(trans_targetcood)
    

    def transfeatures_enu(self,cad_features,transparams,default_z:float = 0.0):
        """将feature中的所有点位进行转换"""

        trans_features = list()
        for feature in cad_features["features"]:
            newteature = feature
            if feature["geometry"]["type"]=="Polygon":
                new_coords = list()
                for point in feature["geometry"]['coordinates'][0]:
                    xb, yb, zb = self.transform_point(point[0], point[1], default_z, transparams)
                    new_coords.append([xb, yb])
                newteature["geometry"]["coordinates"]= [new_coords]
            elif feature["geometry"]["type"]=="Point":
                point = feature["geometry"]['coordinates']
                xb, yb, zb = self.transform_point(point[0], point[1], default_z, transparams)
                new_coords=[xb, yb]
                newteature["geometry"]["coordinates"]= new_coords
            elif feature["geometry"]["type"]=="LineString":
                new_coords = list()
                for point in feature["geometry"]['coordinates']:
                    xb, yb, zb = self.transform_point(point[0], point[1], default_z, transparams)
                    new_coords.append([xb, yb])
                newteature["geometry"]["coordinates"]= new_coords
            else:
                
                logger.error(f"Unknow type {feature['geometry']['type']}")
                continue
            
            trans_features.append(newteature)
        return trans_features
    
    def transfeatures_geometry(self,cad_features_enu,default_z:float = 0.0):
        """转换回经纬度"""

        trans_features = list()
        for feature in cad_features_enu:
            newteature = feature
            if feature["geometry"]["type"]=="Polygon":
                new_coords = list()
                for point in feature["geometry"]['coordinates'][0]:
                    
                    lat,lon,alt = pymap3d.enu2geodetic(point[0], point[1], default_z, self.ref_gps[1],self.ref_gps[0],default_z)

                    new_coords.append([lon, lat])
                newteature["geometry"]["coordinates"]= [new_coords]
            elif feature["geometry"]["type"]=="Point":
                point = feature["geometry"]['coordinates']
                lat,lon,alt = pymap3d.enu2geodetic(point[0], point[1], default_z, self.ref_gps[1],self.ref_gps[0],default_z)

                new_coords=[lon, lat]
                newteature["geometry"]["coordinates"]= new_coords
            elif feature["geometry"]["type"]=="LineString":
                new_coords = list()
                for point in feature["geometry"]['coordinates']:
                    lat,lon,alt = pymap3d.enu2geodetic(point[0], point[1], default_z, self.ref_gps[1],self.ref_gps[0],default_z)
                    new_coords.append([lon, lat])
                newteature["geometry"]["coordinates"]= new_coords
            else:
                
                logger.error(f"Unknow type {feature['geometry']['type']}")
                continue
            
            trans_features.append(newteature)
        return trans_features

 
if __name__ == "__main__":
    # 加载match数据
    file = "data/data.json"
    with open(file,'r')as f:
        match_data = json.load(f)

    cad_data_file = "data/clean.geojson"
    with open(cad_data_file,'r')as f:
        cad_data = json.load(f)


    transer = CoorsTrans()
    result = transer(match_data,cad_data)


    result_features = {
    "type": "FeatureCollection",
    "features": result
    }

    result_file = "trans_cad.geojson"
    with open(result_file,'w')as fp:
        json.dump(result_features,fp)