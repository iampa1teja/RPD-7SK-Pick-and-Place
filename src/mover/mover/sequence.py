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
        r   = a.get('roll', 0.0)
        p   = a.get('pitch', 1.57)
        yaw = a.get('yaw', 0.0)
        approach = a.get('approach_height', 0.20)
        grasp    = a.get('grasp_height', 0.10)
        gripper_val = a['gripper_val']

        if not self.bot.go_to_named_pose('ready'):
            print('Pick failed at step: go_to_named_pose(ready)')
            return False
        
        self.bot.set_gripper(0.0)

        if not self.bot.go_to_pose(x, y, z + approach, r, p, yaw):
            print(f'Pick failed at step: approach to ({x}, {y}, {z + approach})')
            return False

        self._wait(2.0)

        if not self.bot.go_to_pose(x, y, z + grasp, r, p, yaw):
            print(f'Pick failed at step: grasp to ({x}, {y}, {z + grasp})')
            return False

        if not self._wait(1.0):
            print('Pick failed at step: wait before gripper close')
            return False

        if not self.bot.set_gripper(gripper_val):
            print(f'Pick failed at step: set_gripper({gripper_val})')
            return False

        if not self._wait(0.5):
            print('Pick failed at step: wait after gripper close')
            return False

        return True

    def _place(self, a):
        x, y, z = a['x'], a['y'], a['z']
        r   = a.get('roll', 0.0)
        p   = a.get('pitch', 1.57)
        yaw = a.get('yaw', 0.0)
        approach    = a.get('approach_height', 0.20)
        release = 0.15
        gripper_val = a['gripper_val']

        if not self.bot.go_to_pose(x, y, z + approach, r, p, yaw):
            print(f'Place failed at step: approach to ({x}, {y}, {z + approach})')
            return False

        if not self.bot.go_to_pose(x, y, z+release, r, p, yaw):
            print(f'Place failed at step: descend to ({x}, {y}, {z})')
            return False

        if not self._wait(1.0):
            print('Place failed at step: wait before gripper open')
            return False

        if not self.bot.set_gripper(gripper_val):
            print(f'Place failed at step: set_gripper({gripper_val})')
            return False

        if not self._wait(0.5):
            print('Place failed at step: wait after gripper open')
            return False

        if not self.bot.go_to_named_pose('ready'):
            print('Place failed at step: go_to_named_pose(ready)')
            return False

        return True

    def _wait(self, seconds):
        time.sleep(seconds)
        return True