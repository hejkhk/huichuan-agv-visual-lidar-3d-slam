from robot_api.mock import MockRobotApi


def test_charging_point_is_unique_and_replaceable(tmp_path):
    api = MockRobotApi(tmp_path)
    assert api.set_charging_point("p1").success
    points = api.list_points().data
    assert [p["id"] for p in points if p["is_charging_point"]] == ["p1"]
    assert api.set_charging_point("p2").success
    points = api.list_points().data
    assert [p["id"] for p in points if p["is_charging_point"]] == ["p2"]
