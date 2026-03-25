""" Created at:2026/03/25 09:48:28,@Author: yimy.
description: 该模块用于将停车位与车道中心线构建拓扑关系，将停车位关联到某条中心线上，方便后续进行路径规划和导航等操作。
"""

from shapely.geometry import Polygon, LineString
from shapely import STRtree  # 核心：空间索引（Shapely 2.0+ 内置）
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
import collections

class ParkPlotTopo:
    def __init__(
        self
    ) -> None:
        pass


    def __call__(self, polylines,parklots)  -> None:
        
        # 数据预处理：将输入的车道中心线和停车位数据转换为Shapely对象
        shapely_polylines, shapely_parklots = self.__initdata(polylines, parklots)
        # 构建空间索引并关联停车位到车道中心线
        parklot_to_polyline_match = self.__associate_parklots_to_polylines(shapely_polylines, shapely_parklots)

        self.visualize_results(shapely_parklots,shapely_polylines,parklot_to_polyline_match)




    def __initdata(self, polylines, parklots):
        """该函数用于将输入的车道中心线和停车位数据转换为Shapely对象
        """
        shapely_polylines = [LineString(polyline) for polyline in polylines]
        shapely_parklots = [Polygon(parklot) for parklot in parklots]
        return shapely_polylines, shapely_parklots


    def __associate_parklots_to_polylines(self, polylines, parklots):
        """该函数用于将停车位与最近的车道中心线进行关联
        输入：polylines - 车道中心线的列表，parklots - 停车位的列表
        输出：polyline_plot_match - 中心线编号对应的停车位编号列表
        """
        
        
        polyline_plot_match = collections.defaultdict(list)
        for index, parklot in enumerate(parklots):
            min_dist = float('inf')
            best_idx = -1

            for idx, pl in enumerate(polylines):
                dist = parklot.distance(pl)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = idx

            polyline_plot_match[best_idx].append(index)          

        return polyline_plot_match




    def visualize_results(self,rectangles, polylines, match_results,savepath:str = "result_plot/match.jpg"):

        plt.ioff()
        plt.figure(figsize=(12,10))

        # 绘制原始数据
        for rec_idx , rect in enumerate(rectangles):
            color = plt.cm.jet(rec_idx / len(rectangles))
            coords= list(rect.exterior.coords)
            plt.plot(coords, marker='o',  color = color, linewidth=1, markersize=2)

        for poly_idx , line in enumerate(polylines):
            color = plt.cm.jet(poly_idx / len(polylines))
            linex, liney = line.xy
            plt.plot(linex, liney, marker='o',  color = color, linewidth=1, markersize=2)



        # for pl_idx,parkidxs in match_results.items():
        #     color = plt.cm.jet(pl_idx / len(polylines))
        #     line = polylines[pl_idx]
        #     linex, liney = line.xy
        #     plt.plot(linex, liney, marker='o',  color = color, linewidth=1, markersize=2)

        #     for rect_idx in parkidxs:
        #         rect = rectangles[rect_idx]
        #         coords= list(rect.exterior.coords)
        #         plt.plot(coords, marker='o',  color = color, linewidth=1, markersize=2)

        plt.axis('equal')
        plt.axis('off')
        plt.savefig(savepath, bbox_inches='tight', dpi=800)
        plt.close()




if __name__ == "__main__":
    # 加载中心线数据
    with open("data/simplified_centerlines.geojson", "r") as f:
        data= json.load(f)
    polylines = [item["geometry"]["coordinates"] for item in data["features"]]

    # 加载停车位数据
    parklots = list()
    with open("data/B3P_enu.geojson", "r") as f:
        data = json.load(f)
    for item in data["features"]:
        if item["properties"]["entity_nam"] == "停车位":
            parklots.append(item["geometry"]["coordinates"][0])


    topoer = ParkPlotTopo()
    result = topoer(polylines, parklots)

    print()