"""Import every twin-only node module — fails if the sim-mode strip broke one.

Needs a sourced ROS 2 Jazzy environment (the node modules import rclpy + the
message packages). Also asserts the stripped-out modules are really gone.

    python test/test_imports.py
"""
import importlib
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

NODE_MODULES = [
    'twin_bridge', 'pose_sync', 'map_edit', 'obstacle_mirror', 'mission',
    'sim_battery', 'operator_ui', 'preflight',
    'util', 'grid', 'gz_io', 'ui_web',
]


def test_all_node_modules_import():
    for name in NODE_MODULES:
        importlib.import_module(f'algae_twin.{name}')


def test_stripped_modules_absent():
    import algae_twin
    base = os.path.dirname(algae_twin.__file__)
    for gone in ('demo_link.py', 'world_gen.py'):
        assert not os.path.exists(os.path.join(base, gone)), \
            f'{gone} should have been stripped from the twin-only build'


def test_twin_bridge_is_twin_only():
    # the mode parameter + sim branch are gone; mode is fixed to 'twin'
    import inspect

    from algae_twin import twin_bridge
    src = inspect.getsource(twin_bridge)
    assert "declare_parameter('mode'" not in src
    assert "== 'sim'" not in src
    assert "self.mode = 'twin'" in src


if __name__ == '__main__':
    test_all_node_modules_import()
    test_stripped_modules_absent()
    test_twin_bridge_is_twin_only()
    print('IMPORT TESTS PASS')
