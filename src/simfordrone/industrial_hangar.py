"""加载官方 Isaac Sim Warehouse USD，不以基础几何体伪造真实场景。"""

from __future__ import annotations


class IndustrialHangar:
    """把官方大型仓库作为 stage reference 加入当前 Pegasus World。

    Warehouse 自带货架、叉车、工业材质与灯光。它的依赖由 Isaac Sim 的资产系统按需缓存；
    因此本类不生成圆柱、立方体、人工 PBR 材质或额外的假背景。
    """

    def __init__(self, stage, world, config: dict) -> None:
        self.stage = stage
        self.world = world
        self.config = config

    def build(self) -> None:
        """以 USD reference 载入环境，保留 /World 给两架 UAV 使用。"""

        prim_path = str(self.config["prim_path"])
        asset_url = str(self.config["asset_url"])
        if not prim_path.startswith("/World/"):
            raise ValueError("环境 prim_path 必须位于 /World/ 下，避免覆盖无人机根节点")
        if self.stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f"环境 prim 已存在，拒绝重复加载: {prim_path}")

        environment_prim = self.stage.DefinePrim(prim_path, "Xform")
        if not environment_prim.GetReferences().AddReference(asset_url):
            raise RuntimeError(f"无法引用官方 Isaac 环境资产: {asset_url}")
