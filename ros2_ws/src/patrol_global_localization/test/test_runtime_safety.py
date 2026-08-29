"""Runtime ordering and fail-closed task lifecycle tests."""
import ast
import json
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[4]


def load_server():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "patrol_web_server_runtime_test", PROJECT / "patrol_web" / "server.py")
    server = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(server)
    return server


def test_planner_helpers_are_defined_before_main_call():
    path = PROJECT / "scripts" / "planner_motion_bridge.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    guard = next(node for node in tree.body
                 if isinstance(node, ast.If) and "__main__" in ast.unparse(node.test))
    definitions = {node.name: node.lineno for node in tree.body
                   if isinstance(node, ast.FunctionDef)}
    assert definitions["quat_matrix_from_odom"] < guard.lineno
    assert definitions["tf_buffer_lookup"] < guard.lineno
    assert not [node.name for node in tree.body
                if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                and node.lineno > guard.lineno]


def test_task_exception_always_disables_motion_and_releases_goal(tmp_path, monkeypatch):
    server = load_server()
    data = tmp_path / "data"
    runtime = tmp_path / "runtime"
    package = data / "maps" / "factory"
    package.mkdir(parents=True)
    runtime.mkdir()
    monkeypatch.setattr(server, "DATA", data)
    monkeypatch.setattr(server, "RUNTIME", runtime)
    monkeypatch.setattr(server, "CHECKPOINTS_FILE", data / "checkpoints.json")
    monkeypatch.setattr(server, "ACTIVE_MAP_FILE", data / "active_map.json")
    monkeypatch.setattr(server, "GOAL_FILE", runtime / "goal.json")
    point = {"id": "p1", "name": "P1", "map_name": "factory",
             "coord_version": "v1", "x": 1, "y": 1}
    server.CHECKPOINTS_FILE.write_text(json.dumps([point]), encoding="utf-8")
    server.ACTIVE_MAP_FILE.write_text('{"name":"factory"}', encoding="utf-8")
    (package / "metadata.json").write_text('{"coord_version":"v1"}', encoding="utf-8")
    (runtime / "localization_state.json").write_text(json.dumps({
        "updated_at": server.time.time(), "state": "LOCALIZED",
        "ok_for_navigation": True, "map_id": "factory"}), encoding="utf-8")
    server.GOAL_FILE.write_text('{"id":"old"}', encoding="utf-8")
    motion = []
    monkeypatch.setattr(server, "set_nav_motion",
                        lambda enabled, reason="": motion.append((enabled, reason)))
    monkeypatch.setattr(server, "service_status",
                        lambda: {"navigation": {"running": True}})
    monkeypatch.setattr(server, "robot_state", lambda: {"pose": {"x": 0, "y": 0}})
    monkeypatch.setattr(server, "config", lambda: {"arrive_radius": "invalid"})
    monkeypatch.setattr(server, "issue_goal",
                        lambda _point: {"id": "g1", "x": 1, "y": 1})
    server.TASK_RUN.update(state="running")

    server.execute_steps("run1", {
        "steps": [{"type": "goto", "checkpoint_id": "p1", "timeout": 10}]})

    assert server.TASK_RUN["state"] == "failed"
    assert motion[-1][0] is False
    assert not server.GOAL_FILE.exists()
