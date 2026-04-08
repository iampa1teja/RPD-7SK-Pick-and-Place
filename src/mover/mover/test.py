import rclpy
from rclpy.node import Node
from msgs_pkg.msg import DetectionArray
from mover.client import MoveBotClient
from mover.sequence import SequenceRunner


PICK_OBJECT = "red_cube"  
PLACE_OBJECT = "blue_box"    

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            DetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        self.detections = {}
        self.get_logger().info(f"Subscribed to /detections - Pick: {PICK_OBJECT}, Place: {PLACE_OBJECT}")

    def detection_callback(self, msg):
        """Store detected objects by class name"""
        self.detections = {}
        for detection in msg.detections:
            class_name = detection.class_name
            if class_name not in self.detections:
                self.detections[class_name] = []
            self.detections[class_name].append({
                'x': detection.x,
                'y': detection.y,
                'z': detection.z,
                'confidence': detection.confidence
            })
        
        # Log detections
        self.get_logger().debug(f"Detections: {list(self.detections.keys())}")

def main():
    rclpy.init()
    subscriber = DetectionSubscriber()
    rclpy.spin_once(subscriber, timeout_sec=2.0)
    
    if PICK_OBJECT not in subscriber.detections:
        print(f"Error: {PICK_OBJECT} not detected")
        subscriber.destroy_node()
        rclpy.shutdown()
        return
    
    if PLACE_OBJECT not in subscriber.detections:
        print(f"Error: {PLACE_OBJECT} not detected")
        subscriber.destroy_node()
        rclpy.shutdown()
        return
    
    pick_det = subscriber.detections[PICK_OBJECT][0]
    place_det = subscriber.detections[PLACE_OBJECT][0]
    
    print(f"Detected {PICK_OBJECT} at ({pick_det['x']:.3f}, {pick_det['y']:.3f}, {pick_det['z']:.3f})")
    print(f"Detected {PLACE_OBJECT} at ({place_det['x']:.3f}, {place_det['y']:.3f}, {place_det['z']:.3f})")

    TEST_SEQUENCE = {
        "actions": [
            {
                "type": "pick",
                "x": float(pick_det['x']),
                "y": float(pick_det['y']+0.02),
                "z": float(pick_det['z']),
                "gripper_val": -0.012
            },
            {
                "type": "place",
                "x": float(place_det['x']),
                "y": float(place_det['y']+0.04),
                "z": float(place_det['z']),
                "gripper_val": 0.02
            }
        ]
    }
    
    bot = MoveBotClient()
    runner = SequenceRunner(bot)
    
    print(f"\nRunning sequence: Pick {PICK_OBJECT} and place on {PLACE_OBJECT}...")
    success = runner.execute(TEST_SEQUENCE)
    
    if success:
        print("Sequence completed successfully")
    else:
        print("Sequence failed")
    
    bot.destroy_node()
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()