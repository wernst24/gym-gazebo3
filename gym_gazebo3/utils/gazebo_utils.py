import subprocess
import time
import psutil
import os


class GazeboUtils:
    """
    Utility functions for Gazebo operations
    """
    
    @staticmethod
    def kill_gazebo_processes():
        """Kill all Gazebo-related processes"""
        processes_to_kill = [
            'gazebo', 'gzserver', 'gzclient', 'rosmaster', 
            'roscore', 'robot_state_publisher', 'spawn_entity'
        ]
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                proc_info = proc.info
                if proc_info['name'] and any(name in proc_info['name'].lower() 
                                           for name in processes_to_kill):
                    proc.kill()
                elif proc_info['cmdline'] and any(name in ' '.join(proc_info['cmdline']).lower() 
                                                for name in processes_to_kill):
                    proc.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
                
    @staticmethod
    def launch_gazebo(world_file=None, headless=True, gui=False, verbose=False):
        """Launch Gazebo with specified parameters"""
        env = os.environ.copy()
        
        if headless:
            env['DISPLAY'] = ':99'  # Use virtual display
            
        cmd = ['gazebo']
        
        if world_file:
            cmd.append(world_file)
            
        if not gui:
            cmd.append('--headless')
            
        if verbose:
            cmd.append('--verbose')
            
        try:
            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE if not verbose else None,
                stderr=subprocess.PIPE if not verbose else None
            )
            
            # Wait a bit for Gazebo to start
            time.sleep(3)
            
            return process
            
        except Exception as e:
            print(f"Error launching Gazebo: {e}")
            return None
            
    @staticmethod
    def reset_gazebo_simulation():
        """Reset Gazebo simulation to initial state"""
        try:
            subprocess.run(
                ['gz', 'world', '-r'],
                check=True,
                timeout=5
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"Error resetting Gazebo simulation: {e}")
            
    @staticmethod
    def pause_gazebo():
        """Pause Gazebo simulation"""
        try:
            subprocess.run(
                ['gz', 'world', '-p', '1'],
                check=True,
                timeout=5
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"Error pausing Gazebo: {e}")
            
    @staticmethod
    def unpause_gazebo():
        """Unpause Gazebo simulation"""
        try:
            subprocess.run(
                ['gz', 'world', '-p', '0'],
                check=True,
                timeout=5
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"Error unpausing Gazebo: {e}")