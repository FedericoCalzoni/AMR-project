#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum, auto
from tiago_exam_navigation.navigation_client import EnhancedNavigationClient
from tiago_exam_navigation.manipulation_client import ManipulationClient
import time

class TaskState(Enum):
    IDLE = auto()
    NAV_TO_PICK = auto()
    PICKING = auto()
    NAV_TO_PLACE = auto()
    PLACING = auto()
    RECOVERY = auto()
    COMPLETED = auto()

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # State machine initialization
        self.current_state = TaskState.IDLE
        self.current_marker_index = 0
        self.retry_count = 0
        
        # From exam.pdf requirements
        self.marker_sequence = [
            {'id': 63, 'place_frame': 'place_zone_1'},
            {'id': 582, 'place_frame': 'place_zone_2'}
        ]
        
        # Configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_retries', 3),
                ('nav_timeout', 300),
                ('manip_timeout', 120),
                ('recovery_wait', 5.0)
            ]
        )
        
        # Initialize clients with callback groups
        self.cb_group = ReentrantCallbackGroup()
        self.nav_client = EnhancedNavigationClient()
        self.manip_client = ManipulationClient()
        
        # Start task execution
        self.execute_task()

    def execute_task(self):
        """Main task execution routine"""
        while self.current_marker_index < len(self.marker_sequence):
            marker = self.marker_sequence[self.current_marker_index]
            success = self._process_marker(marker)
            
            if not success:
                if self.retry_count < self.get_parameter('max_retries').value:
                    self.get_logger().warn(f"Retrying marker {marker['id']} ({self.retry_count+1}/{self.get_parameter('max_retries').value})")
                    self.retry_count += 1
                    continue
                
                self.get_logger().error(f"Failed marker {marker['id']} after {self.retry_count} retries")
                self.current_marker_index += 1
                self.retry_count = 0
                continue
            
            self.current_marker_index += 1
            self.retry_count = 0

        self.get_logger().info("All markers processed!")
        self._shutdown()

    def _process_marker(self, marker):
        """Handle full sequence for one marker"""
        pick_frame = f"aruco_marker_{marker['id']}"
        
        # Phase 1: Navigate to pick location
        self._set_state(TaskState.NAV_TO_PICK)
        nav_success = self._execute_with_timeout(
            self.nav_client.navigate_to_frame(pick_frame),
            self.get_parameter('nav_timeout').value
        )
        
        if not nav_success:
            return False

        # Phase 2: Pick object
        self._set_state(TaskState.PICKING)
        pick_success = self._execute_with_timeout(
            self.manip_client.pick_object(pick_frame),
            self.get_parameter('manip_timeout').value
        )
        
        if not pick_success:
            return False

        # Phase 3: Navigate to place location
        self._set_state(TaskState.NAV_TO_PLACE)
        place_success = self._execute_with_timeout(
            self.nav_client.navigate_to_frame(marker['place_frame']),
            self.get_parameter('nav_timeout').value
        )
        
        if not place_success:
            return False

        # Phase 4: Place object
        self._set_state(TaskState.PLACING)
        place_success = self._execute_with_timeout(
            self.manip_client.place_object(marker['place_frame']),
            self.get_parameter('manip_timeout').value
        )
        
        return place_success

    async def _execute_with_timeout(self, coroutine, timeout):
        """Execute coroutine with timeout and error handling"""
        try:
            return await rclpy.task.wait_for_future_complete(
                coroutine,
                timeout_sec=timeout
            )
        except TimeoutError:
            self.get_logger().error(f"Operation timed out after {timeout}s")
            return False
        except Exception as e:
            self.get_logger().error(f"Operation failed: {str(e)}")
            return False

    def _set_state(self, new_state):
        """Handle state transitions and logging"""
        self.get_logger().info(f"State change: {self.current_state.name} → {new_state.name}")
        self.current_state = new_state
        
        if new_state == TaskState.RECOVERY:
            self.get_logger().warn("Entering recovery mode...")
            time.sleep(self.get_parameter('recovery_wait').value)

    def _recovery_procedure(self):
        """Execute system-wide recovery actions"""
        self._set_state(TaskState.RECOVERY)
        
        # 1. Cancel current operations
        self.nav_client.cancel_navigation()
        self.manip_client.home_arm()
        
        # 2. Reset to known good state
        self.manip_client.home_arm()
        self.nav_client.cancel_navigation()
        
        # 3. Clear costmaps (implementation needed)
        self.get_logger().info("Recovery completed")

    def _shutdown(self):
        """Clean shutdown procedure"""
        self.get_logger().info("Shutting down...")
        self.nav_client.destroy()
        self.manip_client.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        executor = MultiThreadedExecutor()
        task_manager = TaskManager()
        executor.add_node(task_manager)
        
        executor.spin()
        
    except KeyboardInterrupt:
        task_manager.get_logger().info("Task interrupted by user")
    finally:
        if rclpy.ok():
            task_manager._shutdown()

if __name__ == '__main__':
    main()