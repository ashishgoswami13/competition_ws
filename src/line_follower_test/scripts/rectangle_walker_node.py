#!/usr/bin/env python3
# encoding: utf-8

"""
Rectangle Walker Node - Makes robot walk in a rectangular pattern
Rectangle dimensions: 20cm x 30cm
No line following - autonomous pattern walking
"""

import rospy
import time
from ainex_kinematics.gait_manager import GaitManager

class RectangleWalker:
    """
    Makes the robot walk in a rectangular pattern
    """
    
    def __init__(self):
        rospy.init_node('rectangle_walker', anonymous=False)
        rospy.loginfo("🔲 Rectangle Walker Starting...")
        
        # Rectangle dimensions (in meters)
        self.side_long = rospy.get_param('~side_long', 0.30)   # 30cm
        self.side_short = rospy.get_param('~side_short', 0.20)  # 20cm
        self.turn_angle = rospy.get_param('~turn_angle', 90)    # 90 degrees for rectangle
        
        # Walking parameters - matching Lesson 4 tutorial (main.py example: x_move_amplitude=0.05)
        self.forward_speed = rospy.get_param('~forward_speed', 0.05)  # 5cm per step (tutorial default)
        self.turn_speed = rospy.get_param('~turn_speed', 10.0)  # 10 degrees per step
        
        # Initialize GaitManager
        rospy.loginfo("⚙️  Initializing GaitManager...")
        self.gait_manager = GaitManager()
        
        # Configure walking parameters - EXACT defaults from Reference/walking_param_sim.yaml
        # These are the proven simulation parameters used in Lesson 4
        self.go_gait_param = self.gait_manager.get_gait_param()
        
        # From walking_param_sim.yaml - Gazebo simulation defaults
        self.go_gait_param['body_height'] = 0.025          # init_z_offset
        self.go_gait_param['step_height'] = 0.015          # z_move_amplitude
        self.go_gait_param['hip_pitch_offset'] = 15        # hip_pitch_offset
        self.go_gait_param['z_swap_amplitude'] = 0.006     # z_swap_amplitude (body vertical oscillation)
        self.go_gait_param['pelvis_offset'] = 0            # pelvis_offset (hip roll)
        
        # DSP parameters from Reference defaults
        # Format: [period_time (ms), dsp_ratio, y_swap_amplitude (m)]
        # Simulation uses MUCH slower period_time: 1500ms vs 400ms for real robot
        self.go_dsp = [1500, 0.3, 0.035]  # Exact values from walking_param_sim.yaml
        self.go_arm_swap = 0  # arm_swing_gain: 0.0 in simulation (disabled)
        
        # Turning gait - slightly different for stability
        self.turn_gait_param = self.gait_manager.get_gait_param()
        self.turn_gait_param['body_height'] = 0.025
        self.turn_gait_param['step_height'] = 0.020        # Slightly higher for turns
        self.turn_gait_param['hip_pitch_offset'] = 15
        self.turn_gait_param['z_swap_amplitude'] = 0.006
        self.turn_gait_param['pelvis_offset'] = 0
        
        # Slower turning for simulation stability
        self.turn_dsp = [1800, 0.35, 0.035]  # Even slower for turns
        self.turn_arm_swap = 0  # No arm swing during turns
        
        # State tracking
        self.total_distance_walked = 0.0
        self.total_angle_turned = 0.0
        self.current_side = 0  # 0=long, 1=short, 2=long, 3=short
        self.laps_completed = 0
        
        rospy.loginfo("✅ Rectangle Walker Ready!")
        rospy.loginfo(f"   Rectangle: {self.side_long*100}cm x {self.side_short*100}cm")
        rospy.loginfo(f"   Forward speed: {self.forward_speed*1000}mm per step")
        
    def initialize_robot(self):
        """Initialize robot to standing position"""
        rospy.loginfo("⏳ Waiting for robot to spawn and stabilize...")
        rospy.sleep(3.0)  # Wait for Gazebo physics to settle
        
        rospy.loginfo("🤖 Executing walk_ready action...")
        try:
            # Use motion_manager to execute walk_ready action (like Reference code)
            from ainex_sdk.action_manager import MotionManager
            motion_manager = MotionManager()
            motion_manager.run_action('walk_ready')
            rospy.sleep(2.0)  # Wait for action to complete
            rospy.loginfo("✅ Walk ready action completed!")
        except Exception as e:
            rospy.logwarn(f"Motion manager not available, using direct pose: {e}")
            # Fallback: Just update pose
            self.gait_manager.update_pose(self.go_gait_param)
            rospy.sleep(2.0)
        
        rospy.loginfo("🎬 Starting rectangle walk in 2 seconds...")
        rospy.sleep(2.0)  # Final countdown before starting
        rospy.loginfo("✅ Robot initialized and ready to walk!")
        
    def walk_forward(self, distance):
        """
        Walk forward for specified distance
        
        Args:
            distance: Distance in meters
        """
        rospy.loginfo(f"🚶 Walking forward {distance*100:.1f}cm...")
        
        # Calculate number of steps needed
        steps_needed = int(distance / self.forward_speed)
        rospy.loginfo(f"   Steps to execute: {steps_needed}")
        
        # Execute the steps (use step_num to walk specific number of steps)
        self.gait_manager.set_step(
            self.go_dsp,
            self.forward_speed,
            0,  # no lateral movement
            0,  # no rotation
            self.go_gait_param,
            arm_swap=self.go_arm_swap,
            step_num=steps_needed  # Walk specific number of steps, then stop
        )
        
        # Calculate how long this will take and wait for completion
        # DSP[0] is duration in ms, each step takes roughly DSP duration + single support
        step_duration = (self.go_dsp[0] / 1000.0) * 1.5  # Conservative estimate
        total_duration = steps_needed * step_duration
        
        rospy.loginfo(f"   Waiting {total_duration:.1f}s for steps to complete...")
        rospy.sleep(total_duration + 0.5)  # Add buffer time
        
        self.total_distance_walked += distance
        rospy.loginfo(f"   ✓ Completed {distance*100:.1f}cm")
        
    def turn_in_place(self, angle):
        """
        Turn in place by specified angle
        
        Args:
            angle: Angle in degrees (positive = counterclockwise)
        """
        rospy.loginfo(f"🔄 Turning {angle}° in place...")
        
        # Calculate number of turn steps needed
        steps_needed = int(abs(angle) / self.turn_speed)
        turn_direction = 1 if angle > 0 else -1
        
        rospy.loginfo(f"   Turn steps to execute: {steps_needed}")
        
        # Execute the turn steps with small forward movement for stability
        self.gait_manager.set_step(
            self.turn_dsp,
            0.004,  # Small forward movement during turn (more natural)
            0.0,    # no lateral movement
            int(-self.turn_speed * turn_direction),  # rotation (negated for GaitManager)
            self.turn_gait_param,
            arm_swap=self.turn_arm_swap,
            step_num=steps_needed  # Turn specific number of steps, then stop
        )
        
        # Wait for turn to complete
        step_duration = (self.turn_dsp[0] / 1000.0) * 1.5
        total_duration = steps_needed * step_duration
        
        rospy.loginfo(f"   Waiting {total_duration:.1f}s for turn to complete...")
        rospy.sleep(total_duration + 0.5)
        
        self.total_angle_turned += angle
        rospy.loginfo(f"   ✓ Turned {angle}°")
        
    def walk_rectangle(self):
        """
        Walk one complete rectangle
        Pattern: Long side -> Turn 90° -> Short side -> Turn 90° -> repeat
        """
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"🔲 Starting Rectangle Lap {self.laps_completed + 1}")
        rospy.loginfo(f"{'='*50}\n")
        
        # Side 1: Long side
        rospy.loginfo("📍 Side 1/4: Long side (30cm)")
        self.walk_forward(self.side_long)
        rospy.sleep(0.5)
        
        # Turn 1: 90 degrees left
        rospy.loginfo("📍 Turn 1/4: 90° left")
        self.turn_in_place(self.turn_angle)
        rospy.sleep(0.5)
        
        # Side 2: Short side
        rospy.loginfo("📍 Side 2/4: Short side (20cm)")
        self.walk_forward(self.side_short)
        rospy.sleep(0.5)
        
        # Turn 2: 90 degrees left
        rospy.loginfo("📍 Turn 2/4: 90° left")
        self.turn_in_place(self.turn_angle)
        rospy.sleep(0.5)
        
        # Side 3: Long side
        rospy.loginfo("📍 Side 3/4: Long side (30cm)")
        self.walk_forward(self.side_long)
        rospy.sleep(0.5)
        
        # Turn 3: 90 degrees left
        rospy.loginfo("📍 Turn 3/4: 90° left")
        self.turn_in_place(self.turn_angle)
        rospy.sleep(0.5)
        
        # Side 4: Short side (back to start)
        rospy.loginfo("📍 Side 4/4: Short side (20cm) - returning to start")
        self.walk_forward(self.side_short)
        rospy.sleep(0.5)
        
        # Turn 4: 90 degrees left (complete the rectangle)
        rospy.loginfo("📍 Turn 4/4: 90° left - back to starting orientation")
        self.turn_in_place(self.turn_angle)
        rospy.sleep(0.5)
        
        self.laps_completed += 1
        
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"✅ Rectangle Lap {self.laps_completed} COMPLETED!")
        rospy.loginfo(f"   Total distance: {self.total_distance_walked*100:.1f}cm")
        rospy.loginfo(f"   Total rotation: {self.total_angle_turned}°")
        rospy.loginfo(f"{'='*50}\n")
        
    def run(self):
        """Main execution loop"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🤖 RECTANGLE WALKER STARTING")
        rospy.loginfo("="*60)
        
        # Initialize robot with proper delays
        self.initialize_robot()
        
        # Get number of laps to complete
        num_laps = rospy.get_param('~num_laps', 1)
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"🎯 MISSION PARAMETERS")
        rospy.loginfo(f"{'='*60}")
        rospy.loginfo(f"   Number of laps: {num_laps}")
        rospy.loginfo(f"   Rectangle size: {self.side_long*100}cm x {self.side_short*100}cm")
        rospy.loginfo(f"   Perimeter: {2*(self.side_long + self.side_short)*100}cm per lap")
        rospy.loginfo(f"   Forward speed: {self.forward_speed*1000}mm per step")
        rospy.loginfo(f"   Turn speed: {self.turn_speed}° per step")
        rospy.loginfo(f"{'='*60}\n")
        
        # Walk the rectangles
        for lap in range(num_laps):
            if rospy.is_shutdown():
                break
            self.walk_rectangle()
            
            if lap < num_laps - 1:
                rospy.loginfo("⏸  Pausing before next lap...")
                rospy.sleep(2.0)
        
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo(f"🏁 MISSION COMPLETE!")
        rospy.loginfo(f"   Laps completed: {self.laps_completed}")
        rospy.loginfo(f"   Total distance: {self.total_distance_walked*100:.1f}cm")
        rospy.loginfo(f"   Total rotation: {self.total_angle_turned}°")
        rospy.loginfo(f"{'='*50}\n")
        
        # Stop the robot
        rospy.loginfo("🛑 Stopping robot...")
        self.gait_manager.stop()


def main():
    try:
        walker = RectangleWalker()
        walker.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Rectangle Walker Terminated")
    except Exception as e:
        rospy.logerr(f"💥 Error in Rectangle Walker: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
