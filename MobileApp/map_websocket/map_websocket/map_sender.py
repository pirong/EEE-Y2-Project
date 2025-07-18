import rclpy
from rclpy.node import Node
import asyncio
import threading
import json
from nav_msgs.msg import OccupancyGrid, Odometry
import websockets

# Global reference to the ROS node
global_node = None

class MapSenderNode(Node):
    def __init__(self):
        super().__init__('map_sender')
        global global_node
        global_node = self

        self.map_data = None
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info('MapSenderNode started.')

        # Start WebSocket server in another thread
        threading.Thread(target=start_websocket_server, daemon=True).start()

    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Convert quaternion to yaw (theta)
        import math
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        theta = math.atan2(siny_cosp, cosy_cosp)


        self.robot_pose = {
            'x': pos.x,
            'y': pos.y,
            'theta': theta
        }

# WebSocket logic
async def websocket_handler(websocket):
    global global_node
    global_node.get_logger().info("WebSocket client connected")

    try:
        while True:
            if global_node.map_data:
                data = {
                'width': global_node.map_data.info.width,
                'height': global_node.map_data.info.height,
                'resolution': global_node.map_data.info.resolution,  # <-- ADD THIS
                'origin': {
                    'x': global_node.map_data.info.origin.position.x,
                    'y': global_node.map_data.info.origin.position.y
                },
                'data': list(global_node.map_data.data),
                'robot_pose': global_node.robot_pose
            }

                await websocket.send(json.dumps(data))
            await asyncio.sleep(0.5)
    except websockets.exceptions.ConnectionClosed:
        global_node.get_logger().info("WebSocket client disconnected")

def start_websocket_server():
    asyncio.run(run_server())

async def run_server():
    async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
        print("WebSocket server running at ws://0.0.0.0:8765")
        await asyncio.Future()

def main(args=None):
    rclpy.init(args=args)
    node = MapSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
