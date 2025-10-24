#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from math import pi as PI
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import threading
import asyncio
import os
import time
import sys
import signal
import numpy as np
script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_arm.hex_device import Hands, Arm, CommandType
from ros_interface import GripperConfig, ArmConfig

@dataclass
class JointParam:
    joint_name: str
    joint_limit: List[float] # [min_pos, max_pos, min_vel, max_vel, min_acc, max_acc]

class HexArmApi:
    def __init__(self):
        self.data_interface = DataInterface(name="xnode_arm")

        # init arm
        arm_config = ArmConfig()
        if Arm._supports_robot_type(self.data_interface.arm_series):
            self.data_interface.arm = Arm(
                self.data_interface.arm_series,
                arm_config.arm_motor_map[self.data_interface.arm_series],
                control_hz = self.data_interface.ros_rate,
                send_message_callback = self.data_interface._pub_ws_down,
                )
            self.data_interface.arm._set_robot_type(self.data_interface.arm_series)
        else:
            self.data_interface.loge(f"Arm series {self.data_interface.arm_series} is not supported, initializing failed.")

        ## reload joints config
        if self.data_interface.joint_config_path is None:
            self.data_interface.logi("Joint config path is not set, using default config.")
        else:
            self.joints = self.data_interface.get_config_from_json(self.data_interface.joint_config_path)

        ## get init pose and step limits
        if self.data_interface.init_pose_path is not None:
            init_config = self.data_interface.get_init_pose_config(self.data_interface.init_pose_path)
            if init_config is not None:
                self.init_pose = init_config.get('init_pose', None)
                self.step_limits = init_config.get('step_limits', None)
                self.data_interface.logi(f"Init pose: {self.init_pose}")
                if self.step_limits is not None:
                    self.data_interface.logi(f"Step limits: {self.step_limits}")
            else:
                self.init_pose = False
                self.step_limits = None
        else:
            self.init_pose = False
            self.step_limits = None

        # init hands (only if gripper_type is valid, not 0/HtInvalid)
        if self.data_interface.gripper_type is not None and self.data_interface.gripper_type != 0:
            gripper_config = GripperConfig()
            self.data_interface.hands = Hands(
                self.data_interface.gripper_type, 
                gripper_config.gripper_motor_map[self.data_interface.gripper_type], 
                control_hz = self.data_interface.ros_rate,
                send_message_callback = self.data_interface._pub_ws_down,
                )
        
async def _run_with_cancellation(coro, stop_event, loop):
    """Wrapper to run coroutine and check for stop event"""
    task = asyncio.create_task(coro)
    try:
        # Run task but check stop event periodically
        while not task.done():
            if stop_event.is_set():
                task.cancel()
                break
            await asyncio.sleep(0.01)
        await task
    except asyncio.CancelledError:
        # Task was cancelled, this is expected
        pass
    except Exception as e:
        print(f"Task exception: {e}")

def run_async_in_thread(coro, stop_event):
    """Helper function to run async coroutine in a thread with a new event loop"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run_with_cancellation(coro, stop_event, loop))
    except Exception as e:
        print(f"Async thread exception: {e}")
    finally:
        # Cancel all remaining tasks
        try:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            # Wait for all tasks to complete cancellation
            if pending:
                loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        except Exception:
            pass
        finally:
            loop.close()

def signal_handler(signum, frame, stop_event, arm_thread, hands_thread, api):
    """Custom signal handler for graceful shutdown"""
    # stop arm
    print("\n[Ctrl-C] Received shutdown signal")
    api.data_interface.arm.stop()
    time.sleep(0.5)

    # Signal threads to stop
    print("[Shutdown] Stopping async threads...")
    stop_event.set()
    
    # Wait for threads to finish with timeout
    if arm_thread and arm_thread.is_alive():
        arm_thread.join(timeout=2.0)
        if arm_thread.is_alive():
            print("[Warning] Arm thread did not stop gracefully")
    
    if hands_thread and hands_thread.is_alive():
        hands_thread.join(timeout=2.0)
        if hands_thread.is_alive():
            print("[Warning] Hands thread did not stop gracefully")
    
    print("[Shutdown] Shutting down ROS interface...")
    api.data_interface.shutdown()
    print("[Shutdown] Complete")
    sys.exit(0)

def main():
    api = HexArmApi()
    
    # Create stop event for graceful shutdown
    stop_event = threading.Event()
    
    # thread spawn
    ## arm thread
    arm_thread = None
    if api.data_interface.arm is not None:
        arm_thread = threading.Thread(target=run_async_in_thread, args=(api.data_interface.arm._periodic(), stop_event))
        arm_thread.daemon = True  # Set as daemon so it doesn't block program exit
        arm_thread.start()
    else:
        api.data_interface.loge("Arm not initialized, skipping arm thread.")
        return

    ## hands thread
    hands_thread = None
    if api.data_interface.hands is not None:
        hands_thread = threading.Thread(target=run_async_in_thread, args=(api.data_interface.hands._periodic(), stop_event))
        hands_thread.daemon = True  # Set as daemon so it doesn't block program exit
        hands_thread.start()

    # wait thread to start
    time.sleep(0.2)

    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(signum, frame, stop_event, arm_thread, hands_thread, api))
    signal.signal(signal.SIGTERM, lambda signum, frame: signal_handler(signum, frame, stop_event, arm_thread, hands_thread, api))
    
    # start arm
    api.data_interface.arm.start()
    # begin loop
    while api.data_interface.ok():
        # init arm
        if api.data_interface.arm is not None and api.data_interface._is_init is True and api.data_interface.arm._has_new_data:
            if api.init_pose is not None and isinstance(api.init_pose, list):
                # Get current position
                current_pos = np.array(api.data_interface.arm.get_motor_positions())
                target_pos = np.array(api.init_pose)
                err = target_pos - current_pos
                
                # Send command to arm
                if api.step_limits is not None:
                    step_limits = np.array(api.step_limits)
                    err = np.clip(err, -step_limits, step_limits)
                next_pos = current_pos + err
                api.data_interface.arm.motor_command(CommandType.POSITION, next_pos.tolist())
                
                # Check if init pose is reached (within tolerance)
                if np.allclose(current_pos, target_pos, atol=0.01):
                    api.data_interface.logi("Init pose reached.")
                    api.data_interface._is_init = False
            else:
                api.data_interface.logi("Init pose is not set or is not a list, skipping init.")
                api.data_interface._is_init = False

        # publish joint states
        if api.data_interface.arm is not None and api.data_interface.arm._has_new_data:
            arm_motor = api.data_interface.arm.get_simple_motor_status()
            api.data_interface.pub_motor_status(arm_motor['pos'], arm_motor['vel'], arm_motor['eff'])

        # publish gripper states
        if api.data_interface.hands is not None and api.data_interface.hands._has_new_data:
            gripper_motor = api.data_interface.hands.get_simple_motor_status()
            api.data_interface.pub_motor_status(gripper_motor['pos'], gripper_motor['vel'], gripper_motor['eff'])

        # sleep
        api.data_interface.sleep()

if __name__ == '__main__':
    main()