""" Created at:2026/03/24 15:20:59,@Author: yimy."""
import json,os
from shapely.geometry import LineString
import matplotlib.pyplot as plt

class CenterlineSimplify:
    def __init__(
        self,
        interpolation_distance=0.25,
    ):
        self.interpolation_distance = interpolation_distance 



    def __call__(self, segment_lines):

        segments = [((seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])) for seg in segment_lines]
        # 生成多段线
        polylines, conn_count, key_nodes = self.generate_independent_polylines(segments)

        # 对多段线分类
        type1_polylines, type2_polylines, type3_polylines = self.classify_polylines(polylines, key_nodes)

        # 简化多段线
        simplified_type1, simplified_type2, simplified_type3 = self.simplify_polylines(type1_polylines, type2_polylines, type3_polylines)

        # 可视化结果
        polylines_to_visualize = simplified_type1 + simplified_type2 + simplified_type3
        self.visualize_polylines(simplified_type1)

        return simplified_type1

        
    def count_endpoint_connections(self,segments):
        """统计每个端点的连接次数，找出关键交点"""
        connection_count = {}
        for start, end in segments:
            connection_count[start] = connection_count.get(start, 0) + 1
            connection_count[end] = connection_count.get(end, 0) + 1
        key_nodes = [point for point, count in connection_count.items() if count >= 3]
        return connection_count, key_nodes

    def generate_independent_polylines(self,segments):
        """生成独立多段线"""
        connection_count, key_nodes = self.count_endpoint_connections(segments)
        unused_segments = set(frozenset(seg) for seg in segments)
        polylines = []

        for node in key_nodes:
            if not unused_segments:
                break
            related_segments = [seg for seg in unused_segments if node in seg]
            for seg in related_segments:
                if seg not in unused_segments:
                    continue

                current_polyline = [node]
                current_point = node
                while True:
                    next_candidate_segs = [s for s in unused_segments if current_point in s]
                    if not next_candidate_segs:
                        break

                    next_seg = next_candidate_segs[0]
                    other_point = next(pt for pt in next_seg if pt != current_point)
                    current_polyline.append(other_point)
                    unused_segments.remove(next_seg)

                    if other_point in key_nodes:
                        break
                    current_point = other_point

                if len(current_polyline) > 1:
                    polylines.append(current_polyline)

        while unused_segments:
            remaining_seg = unused_segments.pop()
            polylines.append(list(remaining_seg))

        return polylines, connection_count, key_nodes

    def classify_polylines(self,polylines, key_nodes):
        """
        对多段线分类：
        - type1: 两端均为关键交点
        - type2: 仅一端为关键交点
        - type3: 两端均非关键交点
        """
        type1 = []  # 两端都是关键交点
        type2 = []  # 仅一端是关键交点
        type3 = []  # 两端都不是关键交点

        for polyline in polylines:
            first_pt = polyline[0]
            last_pt = polyline[-1]
            first_is_key = first_pt in key_nodes
            last_is_key = last_pt in key_nodes

            if first_is_key and last_is_key:
                type1.append(polyline)
            elif first_is_key or last_is_key:
                type2.append(polyline)
            else:
                type3.append(polyline)

        return type1, type2, type3

    def simplify_polylines(self, type1_polylines, type2_polylines, type3_polylines):
        """对不同类型的多段线进行简化处理"""
        # 这里可以根据需要对不同类型的多段线进行不同的简化策略
        # 例如：
        # - type1_polylines: 保持原样或进行适度简化
        # - type2_polylines: 适度简化，保留关键交点
        # - type3_polylines: 可以进行更 aggressive 的简化

        simplified_type1 = [self.simplify_polyline(poly) for poly in type1_polylines]
        simplified_type2 = [self.simplify_polyline(poly) for poly in type2_polylines]
        simplified_type3 = [self.simplify_polyline(poly) for poly in type3_polylines]

        return simplified_type1, simplified_type2, simplified_type3

    def simplify_polyline(self, polyline_coords):
        """简化单条多段线的示例方法，可以根据需要实现具体的简化算法"""
        # Douglas-Peucker 算法或其他线简化算法

        line = LineString(polyline_coords)  
        simplified = line.simplify(self.interpolation_distance, preserve_topology=True)
        new_coords = list(simplified.coords)
        return new_coords
        
    def visualize_polylines(self, polylines,ourdir="result_plot"):
        """可视化多段线的示例方法，可以使用 matplotlib 或其他库进行绘制"""

        os.makedirs(ourdir, exist_ok=True)  

        for i, polyline in enumerate(polylines):
            x, y = zip(*polyline)
            color = plt.cm.jet(i / len(polylines))  # 颜色渐变
            plt.plot(x, y, marker='o',  color = color, linewidth=1, markersize=2)

        plt.title("Simplified Centerlines")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis('equal')
        plt.savefig(f"{ourdir}/中心线筛选与简化.png", dpi=800)
        


if __name__ == "__main__":
    Centerlinefile = "data/centerlines_output.geojson"
    with open(Centerlinefile, 'r') as f:
        data = json.load(f)
    polylines = [feature["geometry"]["coordinates"] for feature in data['features']]
    
    simplifier = CenterlineSimplify(1.5)
    result = simplifier(polylines)

    # 保存第一个为成geojson
    output_geojson = {
        "type": "FeatureCollection",
        "features": []
    }
    for polyline in result:
        feature = {
            "type": "Feature",
            "geometry": {
                "type": "LineString",
                "coordinates": polyline
            },
            "properties": {}
        }
        output_geojson["features"].append(feature)

    with open("data/simplified_centerlines.geojson", 'w') as f:
        json.dump(output_geojson, f, indent=2)