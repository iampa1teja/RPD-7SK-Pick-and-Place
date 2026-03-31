import time


class SequenceRunner:
    def __init__(self, bot):
        self.bot = bot

    def execute(self, json_input: dict) -> bool:
        for action in json_input['actions']:
            if action['type'] == 'pick':
                success = self._pick(action)
            elif action['type'] == 'place':
                success = self._place(action)
            else:
                print(f"Unknown action type: {action['type']}")
                success = False

            if not success:
                print(f"Action failed: {action['type']} — aborting sequence")
                return False
        return True

    def _pick(self, a):
        x, y, z = a['x'], a['y'], a['z']
        r   = a.get('roll', -1.57)
        p   = a.get('pitch', 0.0)
        yaw = a.get('yaw', 0.0)
        approach = a.get('approach_height', 0.15)
        grasp    = a.get('grasp_height', 0.1)
        gripper_val = a['gripper_val']

        return (
            self.bot.go_to_named_pose('ready')                        and
            self.bot.go_to_pose(x, y, z + approach, r, p, yaw)       and
            self.bot.go_to_pose(x, y, z + grasp, r, p, yaw)          and
            self._wait(1.0)                                          and 
            self.bot.set_gripper(gripper_val)                         and
            self._wait(0.5)
        )

    def _place(self, a):
        x, y, z = a['x'], a['y'], a['z']
        r   = a.get('roll', -1.57)
        p   = a.get('pitch', 0.0)
        yaw = a.get('yaw', 0.0)
        approach    = a.get('approach_height', 0.15)
        gripper_val = a['gripper_val']

        return (
            self.bot.go_to_pose(x, y, z + approach, r, p, yaw)       and
            self.bot.go_to_pose(x, y, z, r, p, yaw)                   and
            self._wait(1.0)                                           and 
            self.bot.set_gripper(gripper_val)                         and
            self._wait(0.5)                                            and
            self.bot.go_to_named_pose('ready')
        )

    def _wait(self, seconds):
        time.sleep(seconds)
        return True