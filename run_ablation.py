import os
import sys
import subprocess
import time
import argparse
import yaml

def scorched_earth_cleanup():
    """Deep clean between density batches to prevent zombie accumulation."""
    print("🧹 Executing Surgically Precise Scorched Earth Cleanup...")
    
    kill_commands = [
        # 1. Kill the specific python controllers 
        "pkill -9 -f sim_node_lidar_ros2.py",
        
        # 2. Kill Gazebo executables EXACTLY
        "pkill -9 -x gz",
        "pkill -9 -x ruby",
        "pkill -9 -f 'gz sim'",
        
        # 3. Kill the ROS/Gazebo bridge binaries
        "pkill -9 -x ros_gz_bridge",
        "pkill -9 -x parameter_bridge",
        "pkill -9 -x image_bridge",
        
        # 4. Kill ROS 2 Launch and lingering utility nodes
        "pkill -9 -f static_transform_publisher",
        "pkill -9 -f robot_state_publisher",
        "pkill -9 -f 'ros2 launch'",
        
        # 5. CRITICAL: Reset the ROS 2 DDS Discovery Daemon
        "ros2 daemon stop",
        "pkill -9 -f _ros2_daemon"
    ]
    
    for cmd in kill_commands:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # Wait for OS to release ports and memory
    time.sleep(3.0)

if __name__=="__main__":
    parser = argparse.ArgumentParser(description="Master Supervisor for Ablation Studies")
    parser.add_argument("--config", default="/home/arl/lmf_ws/src/rmf_gz/ablation_config.yaml", help="Path to the ablation YAML config file")
    args = parser.parse_args()

    # Resolve absolute path to ensure downstream scripts can always find it
    config_path = os.path.abspath(os.path.expanduser(args.config))

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"\n[!] CRITICAL ERROR: Config file '{config_path}' not found.")
        print("Please ensure the file exists or pass the correct path using --config")
        sys.exit(1)

    # ─── EXTRACT CONFIGURATION PARAMETERS ──────────────────────────────────────
    ablation_cfg = config.get('ablation', {})
    sim_cfg      = config.get('simulation', {})

    NUM_RUNS        = ablation_cfg.get('runs_per_density', 50)
    DENSITIES       = ablation_cfg.get('densities', [1.5, 1.8, 2.0, 2.5, 3.0])
    EXPERIMENT_PATH = os.path.expanduser(ablation_cfg.get('experiment_script_path', 'src/rmf_gz/run_experiment.py'))
    CONTROLLER_PATH = os.path.expanduser(ablation_cfg.get('controller_script_path', '~/nCBF3D/nCBF3D/G_ROS_interface/sim_node_lidar_ros2.py'))
    
    WORLD_NAME      = sim_cfg.get('world_name', 'random3d')
    TIMEOUT         = str(sim_cfg.get('run_timeout_s', 300.0))
    IS_HEADLESS     = sim_cfg.get('headless', True)

    # ─── EXECUTE BATCHES ───────────────────────────────────────────────────────
    
    # Ensure starting from a clean slate
    scorched_earth_cleanup()
    
    for density in DENSITIES:
        print(f"\n{'='*60}")
        print(f"🚀 STARTING BATCH: Density {density} | Runs: {NUM_RUNS}")
        print(f"{'='*60}")
        
        # Construct the core command list
        cmd = [
            "python3", EXPERIMENT_PATH, CONTROLLER_PATH,
            "--config", config_path,      # Pass absolute config path downstream
            "--runs", str(NUM_RUNS),
            "--world", WORLD_NAME,
            "--radius", str(density),
            "--timeout", TIMEOUT
        ]
        
        # Conditionally append the headless flag
        if IS_HEADLESS:
            cmd.append("--headless")
        
        try:
            # Execute the experiment runner script
            subprocess.run(cmd, check=True)
            
        except subprocess.CalledProcessError as e:
            print(f"\n[!] Batch for density {density} failed with exit code {e.returncode}.")
            
        except KeyboardInterrupt:
            print("\n[!] Meta-supervisor interrupted by user. Aborting entire pipeline...")
            scorched_earth_cleanup()
            sys.exit(1)
            
        # Clean the GPU, RAM, and DDS Network before the next density starts
        scorched_earth_cleanup()
        
    print("\n🎉 ALL ABLATION BATCHES COMPLETE!")