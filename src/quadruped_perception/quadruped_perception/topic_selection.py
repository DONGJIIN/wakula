"""多个候选传感器话题共用的单源锁定规则。

同时处理两路同类型传感器会重复计算并破坏时间窗口，因此节点锁定首个活跃来源；只有当前
来源超过 timeout 不再发布，才允许另一个候选源接管。
"""


def should_accept_source(
    active_source: str,
    candidate_source: str,
    active_age: float,
    switch_timeout: float,
) -> bool:
    """判断样本能否成为活跃来源：接受当前源，或在当前源失联后允许切换。"""
    return (
        not active_source
        or candidate_source == active_source
        or active_age > max(0.0, switch_timeout)
    )
