import json
import os
from process import GenerateFloorMap

if __name__ == "__main__":
    geojson_path = "data/b1.geojson"
    outdir = "floor_result"

    generator = GenerateFloorMap(
        floorID = 1394594054,
        floor_num = 22
    )
    generator(geojson_path,outdir)
