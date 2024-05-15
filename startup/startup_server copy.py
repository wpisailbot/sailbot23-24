import sys
sys.path.append('startup_messages/python/')
import startup_pb2 as ros2_control_pb2
import startup_pb2_grpc as ros2_control_pb2_grpc

import grpc
from concurrent import futures
import asyncio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

class ROS2ControlServicer(ros2_control_pb2_grpc.ROS2ControlServicer):
    def __init__(self):
        self.launch_service = None
        self.logs = []
        self.log_publisher = None
        self.node = None
        self.launch_task = None

    async def Start(self, request, context):
        if self.launch_service and self.launch_task and not self.launch_task.done():
            return ros2_control_pb2.LaunchResponse(success=False, message="A launch is already running")
        
        print("Launching nodes...")
        self.launch_service = LaunchService()
        self.logs.append(f"Starting launch file: {request.launch_file} with arguments: {request.arguments}")
        package_name, launch_file_name = request.launch_file.split()
        package_share_directory = get_package_share_directory(package_name)
        launch_file_path = os.path.join(package_share_directory, 'launch', launch_file_name)

        launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments=dict(arg.split('=') for arg in request.arguments.split() if '=' in arg)
        )
        self.launch_service.include_launch_description(launch_description)
        
        self.launch_task = asyncio.create_task(self.launch_service.run_async())
        print("Nodes launched!")
        return ros2_control_pb2.LaunchResponse(success=True, message="Launch started")

    async def Stop(self, request, context):
        if not self.launch_service:
            return ros2_control_pb2.StopResponse(success=False, message="No launch is currently running")
        
        print("Stopping nodes...")
        await self.launch_service.shutdown()
        if self.launch_task:
            self.launch_task.cancel()
            try:
                await self.launch_task
            except asyncio.CancelledError:
                pass
        self.launch_service = None
        self.launch_task = None
        print("Nodes stopped.")
        self.launch_service = None
        self.launch_task = None
        return ros2_control_pb2.StopResponse(success=True, message="Launch stopped")

    async def StreamLogs(self, request, context):
        for log in self.logs:
            yield ros2_control_pb2.LogMessage(timestamp=time.time(), log=log)
        while True:
            await asyncio.sleep(1)
            if self.log_publisher:
                log_msg = "New log entry"
                self.log_publisher.publish(String(data=log_msg))
                yield ros2_control_pb2.LogMessage(timestamp=time.time(), log=log_msg)

async def serve():
    server = grpc.aio.server(futures.ThreadPoolExecutor(max_workers=10))
    ros2_control_pb2_grpc.add_ROS2ControlServicer_to_server(ROS2ControlServicer(), server)
    server.add_insecure_port('[::]:50052')
    await server.start()
    print("Server started at [::]:50052")
    await server.wait_for_termination()

if __name__ == '__main__':
    rclpy.init()
    node = Node('global_grpc_node')
    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()