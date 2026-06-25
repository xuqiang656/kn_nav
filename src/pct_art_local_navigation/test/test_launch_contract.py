from pathlib import Path


def test_launch_contains_expected_chain_and_no_nav2():
    launch_file = Path(__file__).parents[1] / 'launch' / 'pct_art_local_navigation.launch.py'
    source = launch_file.read_text(encoding='utf-8')
    assert "package='art_planner_ros'" in source
    assert "package='pure_pursuit_planner'" in source
    assert "('/pct_path', '/local_path')" in source
    assert 'SetEnvironmentVariable' in source
    assert '/opt/unitree_robotics/lib' in source
    assert 'plan_to_goal_client.py' not in source
    assert "package='nav2" not in source.lower()
