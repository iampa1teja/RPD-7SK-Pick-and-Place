import rclpy
import json
from mover.client import MoveBotClient
from mover.sequence import SequenceRunner

TEST_SEQUENCE = {
    "actions": [
        {
            "type": "pick",
            "x": 0.2, "y": 0.3, "z": 0.03,
            "gripper_val": -0.02
        },
        {
            "type": "place",
            "x": -0.2, "y": 0.3, "z": 0.03,
            "gripper_val": 0.02
        }
    ]
}

def main():
    rclpy.init()
    bot = MoveBotClient()
    runner = SequenceRunner(bot)

    print("Running test sequence...")
    success = runner.execute(TEST_SEQUENCE)

    if success:
        print("Sequence completed successfully")
    else:
        print("Sequence failed")

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()