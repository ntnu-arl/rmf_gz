#!/usr/bin/env python3
import os
import sys
import subprocess
import time
import argparse
import yaml
import shutil
import copy
from pathlib import Path

# ==============================================================
# LAYER 2: RUNTIME CONFIGURATION MARSHALLER
# ==============================================================
def generate_temp_config(base_cfg: dict, active_params: dict, temp_path: str):
    """
    Transforms the master dict and active 1D params into a flat, temporary YAML
    that sim_runner.py and sim_node_lidar_ros2.py understand natively.
    """
    run_cfg = copy.deepcopy(base_cfg)
    
    # 1. Resolve Controller Flags
    ctrl = active_params['controller']
    run_cfg['USE_HEURISTIC'] = (ctrl == "heuristic")
    run_cfg['USE_BLIND']     = (ctrl == "blind")
    run_cfg['USE_RANDOM']    = (ctrl == "random")
    
    # 2. Resolve Lidar Rate
    run_cfg['LIDAR_RATE'] = float(active_params['lidar_rate'])
    
    # 3. Resolve Sensor Noise
    noise = float(active_params['noise'])
    run_cfg['ADD_LIDAR_NOISE'] = (noise > 0.0)
    run_cfg['SENSOR_NOISE_FACTOR'] = noise
    
    # Write the transaction-isolated config to disk
    with open(temp_path, 'w') as f:
        yaml.dump(run_cfg, f, default_flow_style=False, sort_keys=False)


# ==============================================================
# SYSTEM UTILITIES
# ==============================================================
def scorched_earth_cleanup():
    """Deep clean between parameter batches to prevent zombie accumulation."""
    print("🧹 Executing Surgically Precise Scorched Earth Cleanup...")
    kill_commands = [
        "pkill -9 -f sim_node_lidar_ros2.py",
        "pkill -9 -x gz", "pkill -9 -x ruby", "pkill -9 -f 'gz sim'",
        "pkill -9 -x ros_gz_bridge", "pkill -9 -x parameter_bridge", "pkill -9 -x image_bridge",
        "pkill -9 -f static_transform_publisher", "pkill -9 -f robot_state_publisher", "pkill -9 -f 'ros2 launch'",
        "ros2 daemon stop", "pkill -9 -f _ros2_daemon"
    ]
    for cmd in kill_commands:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)


# ==============================================================
# LAYER 1: 1D PARAMETER SWEEP GRID SUPERVISOR
# ==============================================================
def main():
    parser = argparse.ArgumentParser(description="1D Ablation Sweep Supervisor")
    parser.add_argument("--config", default="src/rmf_gz/ablation_config.yaml", help="Path to master config")
    parser.add_argument("--outdir", default="results", help="Base directory for json results")
    args = parser.parse_args()

    config_path = os.path.abspath(os.path.expanduser(args.config))
    try:
        with open(config_path, 'r') as f:
            master_cfg = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"\n[!] CRITICAL ERROR: Config file '{config_path}' not found.")
        sys.exit(1)

    # Extract Baseline (Default) Values (First index of each list)
    def_ctrl  = master_cfg['controllers'][0]
    def_rate  = master_cfg['lidar_rates'][0]
    def_noise = master_cfg['lidar_noise_scale'][0]
    def_s_rad = master_cfg['ablation'].get('radii', [0.5])[0]

    baseline_params = {
        'controller': def_ctrl,
        'lidar_rate': def_rate,
        'noise': def_noise,
        'sphere_radius': def_s_rad
    }

    # Extract global runtime settings
    num_runs     = master_cfg['ablation'].get('runs_per_density', 3)
    densities    = master_cfg['ablation'].get('densities', [1.5, 2.0, 3.0])
    
    # We now fetch radii here just to define the sweep branch
    radii_list   = master_cfg['ablation'].get('radii', [0.5])
    
    exp_script   = os.path.abspath(os.path.expanduser(master_cfg['ablation']['experiment_script_path']))
    ctrl_script  = os.path.abspath(os.path.expanduser(master_cfg['ablation']['controller_script_path']))
    world_name   = master_cfg['simulation'].get('world_name', 'random3d')
    timeout      = str(master_cfg['simulation'].get('run_timeout_s', 100.0))
    is_headless  = master_cfg['simulation'].get('headless', False)

    # Define the 1D sweep branches (Now including Sphere Radius!)
    sweeps = [
        ("Controller_Ablation",   master_cfg['controllers'],       'controller'),
        ("LidarRate_Ablation",    master_cfg['lidar_rates'],       'lidar_rate'),
        ("Noise_Ablation",        master_cfg['lidar_noise_scale'], 'noise'),
        ("SphereRadius_Ablation", radii_list,                      'sphere_radius')
    ]

    temp_config_path = os.path.abspath("temp_active_run.yaml")
    
    # State tracking for the baseline auto-copy feature
    has_run_baseline = False
    baseline_source_dir = None

    print(f"\n{'='*70}")
    print(f"🎯 INITIALIZING 1D PARAMETER SWEEP")
    print(f"Baseline Defaults -> Ctrl: {def_ctrl} | Rate: {def_rate}Hz | Noise: {def_noise} | Radius: {def_s_rad}m")
    print(f"{'='*70}")

    scorched_earth_cleanup()

    for sweep_name, param_list, param_key in sweeps:
        print(f"\n\n{'#'*70}")
        print(f"🔀 ENTERING SWEEP BRANCH: {sweep_name}")
        print(f"{'#'*70}")

        for val in param_list:
            current_params = baseline_params.copy()
            current_params[param_key] = val
            
            # Define structured output directory for JSONs
            safe_val_str = str(val).replace('.', '_')
            output_dir = Path(args.outdir) / sweep_name / f"{param_key}_{safe_val_str}"
            output_dir.mkdir(parents=True, exist_ok=True)
            
            # --- BASELINE CACHE LOGIC ---
            if current_params == baseline_params:
                if has_run_baseline and baseline_source_dir:
                    print(f"  [⏭️] Skipping {param_key}={val} (Baseline already evaluated).")
                    print(f"  [📁] Copying cached baseline results to {output_dir}")
                    
                    # Programmatically copy the baseline JSONs to the new ablation directory
                    for item in os.listdir(baseline_source_dir):
                        if item.endswith('.json'):
                            shutil.copy(os.path.join(baseline_source_dir, item), os.path.join(output_dir, item))
                    continue
                
                has_run_baseline = True
                baseline_source_dir = output_dir # Save the location for future copies
                run_label = "BASELINE"
            else:
                run_label = f"{param_key.upper()} = {val}"

            print(f"\n{'-'*60}")
            print(f"⚙️ PREPARING BATCH: {run_label}")
            print(f"   State: Ctrl={current_params['controller']}, Rate={current_params['lidar_rate']}, Noise={current_params['noise']}, S_Rad={current_params['sphere_radius']}")
            print(f"{'-'*60}")

            generate_temp_config(master_cfg, current_params, temp_config_path)
            base_output_json = str(output_dir / "results.json")

            # ONLY loop over densities now. The sphere radius is locked for this specific run.
            active_sphere_radius = current_params['sphere_radius']

            for density in densities:
                print(f"\n▶️ Density {density} | Sphere Radius {active_sphere_radius} | Runs: {num_runs}")
                
                cmd = [
                    sys.executable, exp_script, ctrl_script,
                    "--config", temp_config_path, 
                    "--runs", str(num_runs),
                    "--world", world_name,
                    "--radius", str(density),
                    "--sphere_radius", str(active_sphere_radius),
                    "--timeout", timeout,
                    "--output", base_output_json
                ]
                
                if is_headless:
                    cmd.append("--headless")
                
                try:
                    subprocess.run(cmd, check=True)
                except subprocess.CalledProcessError as e:
                    print(f"\n[!] Density {density} with sphere radius {active_sphere_radius} failed with exit code {e.returncode}.")
                except KeyboardInterrupt:
                    print("\n[!] Pipeline interrupted by user. Aborting...")
                    scorched_earth_cleanup()
                    if os.path.exists(temp_config_path): os.remove(temp_config_path)
                    sys.exit(1)
                
                scorched_earth_cleanup()

    if os.path.exists(temp_config_path):
        os.remove(temp_config_path)
        
    print("\n🎉 ALL 1D PARAMETER SWEEPS COMPLETE!")

if __name__=="__main__":
    main()