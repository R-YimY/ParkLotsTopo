
__all__ = ['ProvinceCode', 'MapCode', 'ParkType', 'FloorType', 'ParkInfosCode', 'LinkConnectionType', 'SlotType', 'SlotChargeType','LaneLineColor']


class ProvinceCode:
    beijing = 11
    tianjin = 12
    hebei = 13
    shanxi = 14
    neimenggu = 15
    liaoning = 21
    jilin = 22
    heilongjiang = 23
    shanghai = 31
    jiangsu = 32
    zhejiang = 33
    anhui = 34
    fujian = 35
    jiangxi = 36
    shandong = 37
    henan = 41
    hubei = 42
    hunan = 43
    guangdong = 44
    guangxi = 45
    hainan = 46
    chongqing = 50
    sichuan = 51
    guizhou = 52
    yunnan = 53
    xizang = 54
    shaanxi = 61
    gansu = 62
    qinghai = 63
    ningxia = 64
    xinjiang = 65
    taiwan = 71
    xianggang = 81
    aomen = 82

class MapCode:
    CadMap = 10
    SDMap_LD= 31    
    SDMap_TX= 32
    SDMap_BD= 33


class ParkType:
    unknown = 0
    indoor = 1
    outdoor = 2
    mixed = 3


class FloorType:
    unknown = 0


class ParkInfosCode:
    ParkCode = 10
    FloorCode = 11
    LinkNodeCode = 12
    LinkCode = 13
    JunctionCode = 14
    LaneLineCode = 15  
    SlotCode = 16
    LandMarkCode = 17
    POICode = 18


class LinkConnectionType:
    unknown = 0
    inner_plant = 1
    inner_cross = 2
    outter_connect = 3
    outter = 4



class SlotType:
    unknown = 0
    free = 1
    private = 2

class SlotChargeType:
    unknown = 0
    no_charge = 1
    charge_unknown = 2
    fast_charge = 3
    slow_charge = 4

class SlotLockCode:
    unknown = 0
    no_lock = 1
    has_lock = 2
class SlotLimiterType:
    unknown = 0
    has_height = 2
    no_limit = 1


class LandMarkType:
    unknown = 0
    ground_arrow = 1
    speed_bump = 2
    column = 3


class LaneLineColor:
    unknown = 0
    white = 1
    yellow = 2