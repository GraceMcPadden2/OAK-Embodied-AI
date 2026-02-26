import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import sys
import json
import math
import requests
import os
import re


# ---------------- CONFIG ---------------- #

OLLAMA_URL = os.getenv(
        "OLLAMA_URL",
        "http://172.17.0.1:11434/api/generate")

MODEL_NAME = "llama3"

MAP_X_MIN = -5.0
MAP_X_MAX = 5.0
MAP_Y_MIN = -5.0
MAP_Y_MAX = 5.0

LANDMARKS = {
            "origin": (0.0, 0.0),
                # Add known locations here:
                    # "door": (1.2, -0.4),
                    }

# ---------------------------------------- #


class LLMNav(Node):

        def __init__(self, instruction):
                    super().__init__('llm_local_nav')

                            self.current_pose = None

                                    # Subscribe to odometry
                                            self.create_subscription(
                                                                Odometry,
                                                                            '/odom',
                                                                                        self.pose_callback,
                                                                                                    10
                                                                                                            )

                                                    self._action_client = ActionClient(
                                                                        self,
                                                                                    NavigateToPose,
                                                                                                '/navigate_to_pose'
                                                                                                        )

                                                            self.get_logger().info("Waiting for Nav2 action server...")
                                                                    self._action_client.wait_for_server()
                                                                            self.get_logger().info("Connected to Nav2.")

                                                                                    # Wait until pose received
                                                                                            while rclpy.ok() and self.current_pose is None:
                                                                                                            rclpy.spin_once(self)

                                                                                                                    x, y, yaw = self.query_llm(instruction)
                                                                                                                            self.send_goal(x, y, yaw)

                                                                                                                                def pose_callback(self, msg):
                                                                                                                                            px = msg.pose.pose.position.x
                                                                                                                                                    py = msg.pose.pose.position.y

                                                                                                                                                            q = msg.pose.pose.orientation
                                                                                                                                                                    yaw = math.atan2(
                                                                                                                                                                                        2.0 * (q.w * q.z + q.x * q.y),
                                                                                                                                                                                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                                                                                                                                                                                                            )

                                                                                                                                                                            self.current_pose = (px, py, yaw)

                                                                                                                                                                                def query_llm(self, instruction):

                                                                                                                                                                                            cx, cy, cyaw = self.current_pose

                                                                                                                                                                                                    landmarks_text = "\n".join(
                                                                                                                                                                                                                        [f"{name} = ({pos[0]}, {pos[1]})"
                                                                                                                                                                                                                                         for name, pos in LANDMARKS.items()]
                                                                                                                                                                                                                                )

                                                                                                                                                                                                            prompt = f"""
                                                                                                                                                                                                            You are a robotics navigation assistant.

                                                                                                                                                                                                            The coordinate frame is 'map'.
                                                                                                                                                                                                            Units are meters.

                                                                                                                                                                                                            Current robot pose:
                                                                                                                                                                                                            x = {cx}
                                                                                                                                                                                                            y = {cy}
                                                                                                                                                                                                            yaw = {cyaw}

                                                                                                                                                                                                            Map bounds:
                                                                                                                                                                                                            x in [{MAP_X_MIN}, {MAP_X_MAX}]
                                                                                                                                                                                                            y in [{MAP_Y_MIN}, {MAP_Y_MAX}]

                                                                                                                                                                                                            Known locations:
                                                                                                                                                                                                            {landmarks_text}

                                                                                                                                                                                                            If instruction is relative (e.g., move forward 1 meter),
                                                                                                                                                                                                            compute the new absolute map coordinates.

                                                                                                                                                                                                            Return ONLY valid JSON:
                                                                                                                                                                                                            {{"x": float, "y": float, "yaw": float}}

                                                                                                                                                                                                            Instruction: {instruction}
                                                                                                                                                                                                            """

                                                                                                                                                                                                                    response = requests.post(
                                                                                                                                                                                                                                        OLLAMA_URL,
                                                                                                                                                                                                                                                    json={
                                                                                                                                                                                                                                                                        "model": MODEL_NAME,
                                                                                                                                                                                                                                                                                        "prompt": prompt,
                                                                                                                                                                                                                                                                                                        "stream": False
                                                                                                                                                                                                                                                                                                                    },
                                                                                                                                                                                                                                                                timeout=180
                                                                                                                                                                                                                                                                        )

                                                                                                                                                                                                                            text = response.json()["response"].strip()

                                                                                                                                                                                                                                    # Extract JSON safely
                                                                                                                                                                                                                                            match = re.search(r'\{.*\}', text, re.DOTALL)
                                                                                                                                                                                                                                                    if not match:
                                                                                                                                                                                                                                                                    raise RuntimeError(f"No JSON found in LLM output:\n{text}")

                                                                                                                                                                                                                                                                        data = json.loads(match.group(0))

                                                                                                                                                                                                                                                                                # Clamp to map bounds
                                                                                                                                                                                                                                                                                        x = max(min(float(data["x"]), MAP_X_MAX), MAP_X_MIN)
                                                                                                                                                                                                                                                                                                y = max(min(float(data["y"]), MAP_Y_MAX), MAP_Y_MIN)
                                                                                                                                                                                                                                                                                                        yaw = float(data["yaw"])

                                                                                                                                                                                                                                                                                                                self.get_logger().info(
                                                                                                                                                                                                                                                                                                                                    f"LLM output (clamped): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
                                                                                                                                                                                                                                                                                                                                            )

                                                                                                                                                                                                                                                                                                                        return x, y, yaw

                                                                                                                                                                                                                                                                                                                        def send_goal(self, x, y, yaw):

                                                                                                                                                                                                                                                                                                                                    goal_msg = NavigateToPose.Goal()
                                                                                                                                                                                                                                                                                                                                            goal_msg.pose = PoseStamped()

                                                                                                                                                                                                                                                                                                                                                    goal_msg.pose.header.frame_id = "map"
                                                                                                                                                                                                                                                                                                                                                            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

                                                                                                                                                                                                                                                                                                                                                                    goal_msg.pose.pose.position.x = x
                                                                                                                                                                                                                                                                                                                                                                            goal_msg.pose.pose.position.y = y

                                                                                                                                                                                                                                                                                                                                                                                    goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                                                                                                                                                                                                                                                                                                                                                                                            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

                                                                                                                                                                                                                                                                                                                                                                                                    self.get_logger().info(
                                                                                                                                                                                                                                                                                                                                                                                                                        f"Sending goal -> x: {x:.2f}, y: {y:.2f}"
                                                                                                                                                                                                                                                                                                                                                                                                                                )

                                                                                                                                                                                                                                                                                                                                                                                                            self._action_client.send_goal_async(goal_msg)


                                                                                                                                                                                                                                                                                                                                                                                                            def main():
                                                                                                                                                                                                                                                                                                                                                                                                                    rclpy.init()

                                                                                                                                                                                                                                                                                                                                                                                                                        if len(sys.argv) < 2:
                                                                                                                                                                                                                                                                                                                                                                                                                                    print("Usage:")
                                                                                                                                                                                                                                                                                                                                                                                                                                            print('ros2 run llm_nav llm_local_node "Move forward 1 meter"')
                                                                                                                                                                                                                                                                                                                                                                                                                                                    return

                                                                                                                                                                                                                                                                                                                                                                                                                                                    instruction = " ".join(sys.argv[1:])

                                                                                                                                                                                                                                                                                                                                                                                                                                                        node = LLMNav(instruction)
                                                                                                                                                                                                                                                                                                                                                                                                                                                            rclpy.spin(node)


                                                                                                                                                                                                                                                                                                                                                                                                                                                            if __name__ == '__main__':
                                                                                                                                                                                                                                                                                                                                                                                                                                                                    main()

