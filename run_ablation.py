import os
import subprocess
import time

# example use
# python3 src/rmf_gz/run_experiment.py ~/nCBF3D/nCBF3D/G_ROS_interface/sim_node_lidar_ros2.py --runs 50 --world random3d --headless --radius 1.8     --timeout 300

NUM_RUNS = 20 # (Change back to 50 for the real deal!)
DENSITIES = [1.5, 1.8, 2.0, 2.5, 3.0]
EXPERIMENT_PATH = "src/rmf_gz/run_experiment.py" 
# Expand '~' so subprocess finds the absolute path correctly without a shell
CONTROLLER_PATH = os.path.expanduser("~/nCBF3D/nCBF3D/G_ROS_interface/sim_node_lidar_ros2.py")
WORLD_NAME = "random3d"


def scorched_earth_cleanup():
    """Deep clean between density batches to prevent zombie accumulation."""
    print("🧹 Executing Surgically Precise Scorched Earth Cleanup...")
    
    # 1. Kill the specific python controllers (CORRECTED FILENAME)
    os.system("pkill -9 -f sim_node_lidar_ros2.py")  # <-- THIS WAS THE BUG
    os.system("pkill -9 -f pc_to_range.py")
    
    # 2. Kill Gazebo executables EXACTLY
    os.system("pkill -9 -x gz")
    os.system("pkill -9 -x ruby")
    
    # 3. Kill the bridge binaries
    os.system("pkill -9 -x ros_gz_bridge")
    os.system("pkill -9 -x parameter_bridge")
    os.system("pkill -9 -x image_bridge")
    
    # 4. Kill ROS 2 Launch and lingering utility nodes
    os.system("pkill -9 -f static_transform_publisher")
    os.system("pkill -9 -f robot_state_publisher")
    os.system("pkill -9 -f 'ros2 launch'") 
    
    time.sleep(2.0)

if __name__=="__main__":
    # call the execution for each density for the right number of runs
    for density in DENSITIES:
        print(f"\n{'='*60}")
        print(f"🚀 STARTING BATCH: Density {density} | Runs: {NUM_RUNS}")
        print(f"{'='*60}")
        
        # Construct the command list
        cmd = [
            "python3", EXPERIMENT_PATH, CONTROLLER_PATH,
            "--runs", str(NUM_RUNS),
            "--world", WORLD_NAME,
            "--headless",
            "--radius", str(density),
            "--timeout", "300"
        ]
        
        try:
            # Execute the ablation supervisor script
            subprocess.run(cmd, check=True)
            
        except subprocess.CalledProcessError as e:
            print(f"\n[!] Batch for density {density} failed with exit code {e.returncode}.")
            
        except KeyboardInterrupt:
            print("\n[!] Meta-supervisor interrupted by user. Aborting entire pipeline...")
            scorched_earth_cleanup()
            break
            
        # Clean the GPU and RAM before the next density starts
        scorched_earth_cleanup()
        
    print("\n🎉 ALL ABLATION BATCHES COMPLETE!")