"""Check launch parameter forwarding without requiring ROS or PyYAML."""

import ast
from pathlib import Path
import unittest


class LaunchParameterTests(unittest.TestCase):
    def test_experiment_settings_reach_each_robot(self):
        root = Path(__file__).resolve().parents[3]
        tree = ast.parse((root / 'rviz/launch/multi_robot_frontier_explorer.launch.py').read_text())
        selected = [node for node in tree.body if (
            isinstance(node, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == '_LITE_PARAM_KEYS' for t in node.targets)
        ) or (isinstance(node, ast.FunctionDef) and node.name == '_frontier_params')]
        settings = {
            'selection_strategy': 'nearest', 'path_occ_threshold': 90,
            'frontier_assignment': 'robot_rank', 'information_occ_threshold': 50,
            'sensor_range_m': 4.0, 'gain_distance_weight': 2.0, 'gain_max_viewpoints': 7,
            'gain_threshold_ratio': 0.5, 'gain_region_cap': 8000,
            'turn_penalty_m': 0.25, 'hysteresis_bonus_m': 1.5,
            'wedge_detect_radius_m': 0.75, 'max_wedge_cycles': 5,
            'goal_preempt_improvement_m': 3.0,
            'goal_preempt_utility_ratio': 1.5,
        }
        scope = {'_load_yaml': lambda _: {'frontier_explorer': {
            'ros__parameters': {**settings, 'unsupported_parameter': True}}}}
        exec(compile(ast.Module(body=selected, type_ignores=[]), '<launch>', 'exec'), scope)
        for index, robot in enumerate(('robot1', 'robot2', 'robot3')):
            params = scope['_frontier_params']('unused.yaml', robot, True, (255, 0, 0),
                                               robot_index=index, team_size=3)
            self.assertEqual(params['robot_index'], index)
            self.assertEqual(params['team_size'], 3)
            for key, value in settings.items():
                self.assertEqual(params.get(key), value, key)
            self.assertNotIn('unsupported_parameter', params)
            self.assertEqual(params['costmap_topic'], f'/{robot}/global_costmap/costmap')
            self.assertEqual(params['reservation_topic'], f'/{robot}/explore/reservation')
            self.assertEqual(params['reservation_peer_topics'][index], '')
            self.assertEqual(len(params['reservation_peer_topics']), 3)
            self.assertEqual(params['information_map_topic'], f'/{robot}/nav_map')
            self.assertEqual(params['robot_base_frame'], f'{robot}/base_footprint')


if __name__ == '__main__':
    unittest.main()
