"""场景环境层：加载官方 Isaac Sim USD 环境，并验证它确实载入了物体。

此处只负责“把一个真实室内环境放进 stage”。灯光由 ``simfordrone.lighting`` 单独负责，
避免把“换场景”和“调光照”两件事耦合在一个开关里。
"""

from __future__ import annotations

import time
from typing import Any, Mapping


#: 允许出现在 ``environment`` 段中的键。拼错的键必须报错，而不是被静默忽略——
#: 之前的失败模式正是“YAML 看起来改了，场景却还是一片空网格”。
#: 光照作为子段留在同一处，便于“换场景时一起检查光照”，但由 lighting 模块独立解析。
ALLOWED_KEYS = ("preset", "asset_url", "prim_path", "lighting")


def resolve_environment_asset(
    config: Mapping[str, Any],
    environments: Mapping[str, str] | None = None,
) -> tuple[str, str]:
    """把 YAML 的环境选择解析成 ``(标签, USD 路径)``。

    ``preset`` 与 ``asset_url`` **只能二选一**：同时给出会造成“以为换了场景、实际没换”
    的歧义，因此直接拒绝。

    ``preset`` 取自 Pegasus 维护的 Isaac 环境表（``SIMULATION_ENVIRONMENTS``），
    这样 Isaac 升级后资产根路径由上游给出，不需要在本项目里写死 ``Assets/Isaac/5.1``。
    """

    unknown = sorted(set(config) - set(ALLOWED_KEYS))
    if unknown:
        raise ValueError(f"environment 段含未知参数 {unknown}；允许: {list(ALLOWED_KEYS)}")

    preset = config.get("preset")
    asset_url = config.get("asset_url")
    preset_name = str(preset).strip() if preset is not None else ""
    explicit_url = str(asset_url).strip() if asset_url is not None else ""

    if preset_name and explicit_url:
        raise ValueError("environment.preset 与 environment.asset_url 只能设置其中一个")
    if not preset_name and not explicit_url:
        raise ValueError("environment 必须设置 preset 或 asset_url 之一")

    if not preset_name:
        return explicit_url, explicit_url

    if environments is None:
        # 延迟导入：纯解析函数不应依赖 Isaac。测试里注入表即可离线校验。
        from pegasus.simulator.params import SIMULATION_ENVIRONMENTS

        environments = SIMULATION_ENVIRONMENTS
    if preset_name not in environments:
        available = ", ".join(sorted(environments))
        raise ValueError(f"未知环境预设 {preset_name!r}；可用预设: {available}")
    return preset_name, str(environments[preset_name])


class IndustrialHangar:
    """把官方室内环境作为 USD reference 加入当前 Pegasus World。

    环境自带货架、叉车、工业材质等物体；本项目不生成圆柱、立方体或人工 PBR 材质
    去伪造背景。默认预设为仓库类环境，可按需在 YAML 中切换其它官方预设。
    """

    def __init__(self, stage, world, config: Mapping[str, Any]) -> None:
        self.stage = stage
        self.world = world
        self.config = config
        #: build() 之后为实际载入的 prim 数量，供启动日志作为“不是空场景”的证据。
        self.loaded_prim_count = 0
        self.asset_label = ""
        #: 远程环境首次下载依赖可能较慢；超过该时间仍未展开即视为失败。
        self.reference_timeout_s = 300.0

    def build(self) -> str:
        """以 USD reference 载入环境，保留 ``/World`` 给两架 UAV 使用，返回环境标签。"""

        prim_path = str(self.config["prim_path"])
        if not prim_path.startswith("/World/"):
            raise ValueError("环境 prim_path 必须位于 /World/ 下，避免覆盖无人机根节点")
        if self.stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f"环境 prim 已存在，拒绝重复加载: {prim_path}")

        self.asset_label, asset_url = resolve_environment_asset(self.config)
        environment_prim = self.stage.DefinePrim(prim_path, "Xform")
        if not environment_prim.GetReferences().AddReference(asset_url):
            raise RuntimeError(f"无法引用官方 Isaac 环境资产: {asset_url}")

        self.loaded_prim_count = self._wait_for_reference(environment_prim, asset_url)
        if self.loaded_prim_count <= 0:
            # 引用登记成功但内容始终没展开（网络不可达、资产路径失效等）时，
            # 场景只会剩一片空地——这正是“环境没变、没有物体”的表象。
            # 与其让实验在空场景里跑完，不如在这里失败。
            raise RuntimeError(
                f"环境资产在等待后仍未载入任何子 prim，场景为空: "
                f"{self.asset_label} -> {asset_url}"
            )
        return self.asset_label

    def _wait_for_reference(self, environment_prim, asset_url: str) -> int:
        """等待 USD 引用真正展开，返回子 prim 数量。

        ``AddReference`` 只是**登记**引用：本地文件可以当帧合成，而 http(s) 资产由
        omni.client 异步取回（首次还要写入 Omniverse 缓存）。若在当帧就点数，得到的
        必然是 0——这正是“配置写了 Warehouse，画面里却什么都没有”的根因。
        因此这里主动推进应用更新循环，直到引用展开或超时。
        """

        from pxr import Usd
        import omni.kit.app

        app = omni.kit.app.get_app()
        deadline = time.monotonic() + self.reference_timeout_s
        next_report = time.monotonic() + 10.0
        while True:
            try:
                environment_prim.Load()
            except Exception:
                # Load() 在资产尚未取回时会失败，这属于等待期间的正常状态。
                pass
            # PrimRange 包含根 prim 自身，因此子物体数量要减去 1。
            count = sum(1 for _ in Usd.PrimRange(environment_prim)) - 1
            if count > 0:
                return count
            now = time.monotonic()
            if now >= deadline:
                return 0
            if now >= next_report:
                # 首次加载远程环境需要下载依赖，日志里要看得见“在等什么”，
                # 否则长时间静止会被误判成死锁。
                print(f"[scene] 等待环境资产展开: {asset_url}", flush=True)
                next_report = now + 10.0
            app.update()
