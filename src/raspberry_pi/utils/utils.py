class Utils:
    PI_MRAD = 3141.592653
    MRAD_DEG = 17.4532925199
    @staticmethod
    def normalize_mrad(mrad: int) -> int:
        return (mrad + Utils.PI_MRAD) % (2*Utils.PI_MRAD) - Utils.PI_MRAD

    @staticmethod
    def mrad_to_deg(mrad: int) -> float:
        return mrad / Utils.MRAD_DEG
    
    @staticmethod
    def deg_to_mrad(deg: float) -> int:
        return int(deg * Utils.MRAD_DEG)