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
import psutil
import signal
import logging
from io import StringIO

class ROS2ControlServicer(ros2_control_pb2_grpc.ROS2ControlServicer):
    def __init__(self):
        self.launch_service = None
        self.logs = StringIO()
        self.log_publisher = None
        self.node = None
        self.launch_task = None
        self.log_handler = logging.StreamHandler(self.logs)
        self.log_handler.setLevel(logging.INFO)

        sys.stdout = self.log_handler.stream
        sys.stderr = self.log_handler.stream

        # Configure rclpy logging to use the Python logging system
        rclpy.logging.initialize()
        rclpy_logging_logger = rclpy.logging.get_logger('ros2_control')
        rclpy_logging_logger.set_level(rclpy.logging.LoggingSeverity.INFO)
        for handler in logging.getLogger().handlers:
            rclpy_logging_logger.add_handler(handler)

    async def Start(self, request, context):
        if self.launch_service and self.launch_task and not self.launch_task.done():
            return ros2_control_pb2.LaunchResponse(success=False, message="A launch is already running")
        
        print("Launching nodes...")
        self.launch_service = LaunchService()
        self.logs.write(f"Starting launch file: {request.launch_file} with arguments: {request.arguments}\n")
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

    def get_node_pids(self):
        pids = []
        # Find the PIDs of child processes of the launch service
        launch_service_pid = os.getpid()
        for proc in psutil.process_iter(['pid', 'ppid']):
            if proc.info['ppid'] == launch_service_pid:
                pids.append(proc.info['pid'])
        return pids

    async def Stop(self, request, context):
        if not self.launch_service:
            return ros2_control_pb2.StopResponse(success=False, message="No launch is currently running")

        node_pids = self.get_node_pids()

        print("Stopping nodes...")
        if node_pids:
            for pid in node_pids:
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

        if self.launch_task:
            self.launch_task.cancel()
            try:
                await self.launch_task
            except asyncio.CancelledError:
                pass
            
        self.launch_service = None
        self.launch_task = None
        print("Nodes stopped.")
        return ros2_control_pb2.StopResponse(success=True, message="Launch stopped")

    async def StreamLogs(self, request, context):
        print("Got log stream request")
        last_read_pos = self.logs.tell()
        while True:
            await asyncio.sleep(0.1)
            self.logs.seek(last_read_pos)
            new_logs = self.logs.read()
            last_read_pos = self.logs.tell()

            # Process new logs line-by-line
            for line in new_logs.splitlines():
                yield ros2_control_pb2.LogMessage(log=line)

async def serve():
    server = grpc.aio.server(
    futures.ThreadPoolExecutor(max_workers=10),
    options=[
        ('grpc.keepalive_time_ms', 10000),
        ('grpc.keepalive_timeout_ms', 5000),
        ('grpc.keepalive_permit_without_calls', True),
        ('grpc.http2.max_pings_without_data', 0),
        ('grpc.http2.min_time_between_pings_ms', 10000),
        ('grpc.http2.min_ping_interval_without_data_ms', 10000),
    ]
)
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