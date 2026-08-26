from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_high_rate_transport_has_small_bounded_qos():
    lio = (ROOT / "fastlio2/src/lio_node.cpp").read_text(encoding="utf-8")
    pgo = (ROOT / "pgo/src/pgo_node.cpp").read_text(encoding="utf-8")
    localizer = (ROOT / "localizer/src/localizer_node.cpp").read_text(encoding="utf-8")
    assert "SensorDataQoS().keep_last(5)" in lio
    assert "SensorDataQoS().keep_last(2)" in lio
    assert "10000" not in lio
    assert "10000" not in pgo
    assert "SensorDataQoS().keep_last(5)" in pgo
    assert "SensorDataQoS().keep_last(5)" in localizer


def test_large_products_and_histories_are_bounded_or_disabled():
    lio = (ROOT / "fastlio2/src/lio_node.cpp").read_text(encoding="utf-8")
    lio_config = (ROOT / "fastlio2/config/lio.yaml").read_text(encoding="utf-8")
    pgo = (ROOT / "pgo/src/pgo_node.cpp").read_text(encoding="utf-8")
    localizer_config = (ROOT / "localizer/config/localizer.yaml").read_text(encoding="utf-8")
    assert "max_path_poses" in lio and "poses.erase" in lio
    assert "publish_world_cloud: false" in lio_config
    assert "publish_map_cloud: false" in localizer_config
    assert "max_visualized_loops" in pgo
