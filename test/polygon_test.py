from shapely.geometry import Polygon, LineString
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.colors import ListedColormap

# ===================== 1. 配置参数 =====================
RECT_WIDTH = 2.5    # 矩形固定宽度
RECT_LENGTH = 5.0   # 矩形固定长度
NUM_RECTS = 5       # 矩形数量（可修改）
NUM_POLYLINES = 3   # 多段线数量（可修改）
SAVE_PATH = "矩形_多段线_匹配结果.png"  # 可视化图片保存路径

# 配色方案（区分不同矩形-多段线匹配对）
COLORS = ["#F80808", "#00A89D", "#2600FA", "#F6FF00", "#737373", "#000000", "#7B6B2A"]

# ===================== 2. 生成【互不重叠】的矩形 =====================
def create_axis_aligned_rect(center_x, center_y):
    """通过中心点生成轴对齐矩形（固定尺寸2.5x5）"""
    half_w = RECT_WIDTH / 2
    half_l = RECT_LENGTH / 2
    coords = [
        (center_x - half_l, center_y - half_w),
        (center_x + half_l, center_y - half_w),
        (center_x + half_l, center_y + half_w),
        (center_x - half_l, center_y + half_w),
        (center_x - half_l, center_y - half_w)
    ]
    return Polygon(coords)

def generate_non_overlapping_rects(num):
    """生成互不重叠的矩形"""
    rects = []
    while len(rects) < num:
        cx = random.uniform(0, 50)
        cy = random.uniform(0, 50)
        new_rect = create_axis_aligned_rect(cx, cy)
        
        # 不重叠校验
        is_valid = True
        for rect in rects:
            if new_rect.intersects(rect):
                is_valid = False
                break
        if is_valid:
            rects.append(new_rect)
    return rects

# ===================== 3. 定义多段线 =====================
def generate_sample_polylines(num):
    """生成测试用多段线（替换为你的实际坐标）"""
    polylines = [
        LineString([(1,1), (5,8), (10,3)]),
        LineString([(15,20), (25,25)]),
        LineString([(30,5), (32,12), (28,18), (35,22)])
    ]
    return polylines

# ===================== 4. 核心：匹配最近多段线 =====================
def find_closest_polyline(rect, polylines):
    min_distance = float('inf')
    closest_idx = -1
    for idx, pl in enumerate(polylines):
        dist = rect.distance(pl)
        if dist < min_distance:
            min_distance = dist
            closest_idx = idx
    return closest_idx, min_distance

# ===================== 5. 新增：可视化+保存图片 =====================
def visualize_and_save(rectangles, polylines, match_results):
    """
    绘制匹配结果并保存图片
    :param match_results: 列表，每个元素为(矩形索引, 最近多段线索引, 距离)
    """
    plt.figure(figsize=(12, 10), dpi=100)  # 画布大小+高清分辨率
    ax = plt.gca()

    # 绘制所有多段线（灰色底色）
    for idx, pl in enumerate(polylines):
        x, y = pl.xy
        plt.plot(x, y, color='gray', linewidth=2, alpha=0.6, label=f'多段线{idx+1}' if idx == 0 else "")

    # 绘制矩形 + 对应最近多段线（同色高亮）
    legend_handles = []
    for rect_idx, pl_idx, dist in match_results:
        color = COLORS[rect_idx % len(COLORS)]
        
        # 绘制矩形
        rect = rectangles[rect_idx]
        rect_coords = list(rect.exterior.coords)
        mpl_poly = MplPolygon(rect_coords, facecolor=color, alpha=0.4, edgecolor=color, linewidth=3)
        ax.add_patch(mpl_poly)
        
        # 高亮绘制该矩形的最近多段线
        pl = polylines[pl_idx]
        x_pl, y_pl = pl.xy
        plt.plot(x_pl, y_pl, color=color, linewidth=4, label=f'矩形{rect_idx+1} ↔ 多段线{pl_idx+1}\n距离:{dist:.2f}')

    # 图表设置
    plt.title(f'矩形(2.5×5)与最近多段线匹配结果\n(保存路径：{SAVE_PATH})', fontsize=14, fontweight='bold')
    plt.xlabel('X 坐标', fontsize=12)
    plt.ylabel('Y 坐标', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')  # 等比例坐标轴（保证矩形不变形）
    plt.legend(loc='upper right', fontsize=10)
    plt.tight_layout()

    # 保存图片（关键！）
    plt.savefig(SAVE_PATH, bbox_inches='tight', dpi=300)
    plt.close()  # 关闭画布，释放内存
    print(f"\n✅ 可视化结果已保存至：{SAVE_PATH}")

# ===================== 6. 执行主逻辑 =====================
if __name__ == "__main__":
    # 1. 生成数据
    rectangles = generate_non_overlapping_rects(NUM_RECTS)
    polylines = generate_sample_polylines(NUM_POLYLINES)
    
    # 2. 匹配计算
    print("===== 矩形匹配最近多段线结果 =====")
    match_results = []
    for rect_idx, rect in enumerate(rectangles):
        pl_idx, dist = find_closest_polyline(rect, polylines)
        match_results.append((rect_idx, pl_idx, dist))
        print(f"矩形{rect_idx+1} → 最近多段线{pl_idx+1} | 最小距离：{dist:.2f}")
    
    # 3. 可视化并保存
    visualize_and_save(rectangles, polylines, match_results)