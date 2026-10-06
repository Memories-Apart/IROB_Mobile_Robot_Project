"""ROS-free checks of the real tree builder, using simulated observations/actions.

Run with a Python environment containing py_trees:
    python test/test_mission_tree.py
This validates tree control flow, not ROS clients or physical manipulation.
"""
import ast
from pathlib import Path

import py_trees


def load_tree_classes():
    source = Path(__file__).parents[1] / 'warehouse_inventory_robot/mission_node.py'
    wanted = {'ConditionBehavior', 'ActionBehavior', 'UndockBehavior', 'MissionNode'}
    parsed = ast.parse(source.read_text())
    # Retain real class bodies; skip ROS imports and never call Node.__init__.
    subset = ast.Module(
        body=[n for n in parsed.body if isinstance(n, ast.ClassDef) and n.name in wanted],
        type_ignores=[],
    )
    namespace = {'py_trees': py_trees, 'Node': object}
    exec(compile(subset, str(source), 'exec'), namespace)
    return namespace


def test_tree():
    namespace = load_tree_classes()
    status = py_trees.common.Status

    class DemoMission(namespace['MissionNode']):
        def __init__(self):
            self.grade = 'e'
            self.position = 'home'
            self.held = False
            self.delivered = False
            self.safe = True
            self.arm_ready = True
            self.release_started = False
            self.fail_navigation = False
            self.events = []

        def cube_at_destination(self):
            return self.delivered

        def holding_cube(self):
            return self.held

        def at_pose(self, pose):
            return self.position == pose

        def is_undocked(self):
            return True

        def arm_safe(self):
            return self.safe

        def move_arm_safe(self):
            self.events.append('arm-safe')
            if not self.arm_ready:
                return status.RUNNING
            self.safe = True
            return status.SUCCESS

        def go_to_pose(self, pose):
            self.events.append('navigate-' + pose)
            if self.fail_navigation:
                return status.FAILURE
            if pose == 'source':
                self.position = pose
                return status.SUCCESS
            return status.RUNNING

        def pick_cube(self):
            self.events.append('pick')
            self.held = True
            self.safe = False
            return status.SUCCESS

        def place_cube(self):
            self.events.append('place')
            self.held = False
            if not self.release_started:
                self.release_started = True
                return status.RUNNING
            self.delivered = True
            return status.SUCCESS

    mission = DemoMission()
    tree = mission.build_tree('source', 'destination')
    tree.tick()
    assert tree.root.status == status.RUNNING
    assert mission.events == ['navigate-source', 'pick', 'arm-safe', 'navigate-destination']
    nodes = list(tree.root.iterate())
    assert len(nodes) == len({id(node) for node in nodes})
    assert sum(node.memory for node in nodes
               if isinstance(node, (py_trees.composites.Selector,
                                    py_trees.composites.Sequence))) == 1

    # A changed arm condition prevents another navigation step even though
    # the delivery sequence remembers the current destination subtree.
    mission.safe = False
    mission.arm_ready = False
    before = mission.events.count('navigate-destination')
    tree.tick()
    assert mission.events.count('navigate-destination') == before
    navigation = next(n for n in nodes if n.name == 'Navigate to destination')
    assert navigation.status == status.INVALID  # ROS cancellation is still TODO.
    mission.arm_ready = True
    tree.tick()
    assert mission.events.count('navigate-destination') == before + 1

    # Releasing the cube takes two ticks. Do not re-enter pickup during withdrawal.
    mission.position = 'destination'
    tree.tick()
    assert tree.root.status == status.RUNNING and not mission.held
    tree.tick()
    assert tree.root.status == status.SUCCESS
    assert mission.events.count('pick') == 1
    assert mission.events.count('navigate-source') == 1
    assert mission.events.count('place') == 2
    before = list(mission.events)
    tree.tick()
    assert mission.events == before  # Goal condition short-circuits completed work.

    failed = DemoMission()
    failed.fail_navigation = True
    failed_tree = failed.build_tree('source', 'destination')
    failed_tree.tick()
    assert failed_tree.root.status == status.FAILURE
    assert failed.events == ['navigate-source']

    # While transporting, losing the object fails before another motion tick.
    lost = DemoMission()
    lost_tree = lost.build_tree('source', 'destination')
    lost_tree.tick()
    before = list(lost.events)
    lost.held = False
    lost_tree.tick()
    assert lost_tree.root.status == status.FAILURE and lost.events == before

    grade_a = DemoMission()
    grade_a.grade = 'a'
    a_tree = grade_a.build_tree('source', 'destination')
    assert sum(n.name == 'Localization ready?' for n in a_tree.root.iterate()) == 2


if __name__ == '__main__':
    test_tree()
    print('PASS: tree ordering, guards, release progress, failure and A structure')
