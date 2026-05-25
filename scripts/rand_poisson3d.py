import xml.etree.ElementTree as ET
import trimesh
import numpy as np
import scipy as sp
import scipy.stats
import os
import copy
import argparse

PATH_RESOURCES = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../resources')
PATH_WORLDS = os.path.join(PATH_RESOURCES, 'worlds/')
PATH_MODELS = os.path.join(PATH_RESOURCES, 'environment_assets/')

## generate random pose values
def random_poses(n, radius, range, seed=None):
    poisson_engine = sp.stats.qmc.PoissonDisk(
        d=3,
        radius=radius / range,
        ncandidates=100,
        seed=seed,
    )
    return (poisson_engine.random(n) - np.array([0.5, 0.5, 0])) * range

if __name__ == '__main__':
    ## Set up argument parsing
    parser = argparse.ArgumentParser(description='Generate a random SDF world with obstacles using Poisson disk sampling.')
    parser.add_argument(
        '--radius', '-r',
        type=float,
        default=3.0,
        help='Poisson disk radius for obstacle generation (choose density from 1.5 to 3.0). Default is 3.0.'
    )
    parser.add_argument(
        '--sphere_radius', '-sr',
        type=float,
        default=0.5,
        help='Radius of the spherical obstacles (default: 0.5).'
    )
    args = parser.parse_args()

    ## load the SDF file
    tree = ET.parse(os.path.join(PATH_WORLDS, 'empty.sdf'))
    root = tree.getroot()

    ## find the <world> element
    world = root.find('world')
    if world is None:
        raise ValueError('no <world> element found in the SDF file')

    ## uncap simulation speed
    physics = world.find('physics')
    if physics is None:
        # IGNITION GAZEBO FIX: Default physics tags use type 'ignored' as plugins handle the actual math
        physics = ET.SubElement(world, 'physics', name='1ms', type='ignored')
        max_step = ET.SubElement(physics, 'max_step_size')
        max_step.text = '0.001'
    
    # ---------------------------------------------------------
    # SPEED CAP SETTINGS
    # Change TARGET_RTF to test your hardware limit (e.g., 2.0, 3.0, 5.0)
    # ---------------------------------------------------------
    TARGET_RTF = 4.0  
    STEP_SIZE = 0.001 # This must match your <max_step_size>

    # Set real_time_factor (The target speed multiplier)
    rt_factor = physics.find('real_time_factor')
    if rt_factor is None:
        rt_factor = ET.SubElement(physics, 'real_time_factor')
    rt_factor.text = str(TARGET_RTF)

    # Set real_time_update_rate (How many physics steps per real second to achieve the RTF)
    rt_update_rate = physics.find('real_time_update_rate')
    if rt_update_rate is None:
        rt_update_rate = ET.SubElement(physics, 'real_time_update_rate')
    
    update_rate_val = int(TARGET_RTF / STEP_SIZE)
    rt_update_rate.text = str(update_rate_val)

    ## add random obstacles
    poisson_radius = args.radius
    chunks_radius = [poisson_radius] * 3 # choose density form 1.5 to 3.0 for a good distribution
    
    # Generate a random base seed for this specific run
    base_seed = np.random.randint(0, 1000000) 
    
    poisson_domain_size = 8
    x_offset = 2 + poisson_domain_size / 2
    meshes = []
    base_mesh = trimesh.creation.icosphere(subdivisions=2, radius=args.sphere_radius)
    
    for j, r in enumerate(chunks_radius):
        # Use the random base_seed instead of the hardcoded one
        poisson_obs = random_poses(1000, r, poisson_domain_size, seed=base_seed+j)
        n_obstacles = poisson_obs.shape[0]
        print(f'placing {n_obstacles} obstacles in the chunk {j} (Poisson radius: {r})')
        for pos in poisson_obs:
            mesh = base_mesh.copy()
            mesh.apply_translation(pos + [x_offset, 0, 0])
            meshes.append(mesh)

        x_offset += poisson_domain_size + 1

    ## export dae
    # IGNITION GAZEBO FIX: Ensure absolute path resolution
    mesh_filename = os.path.abspath(os.path.join(PATH_MODELS, 'random_spheres.dae'))
    merged_mesh = trimesh.util.concatenate(meshes)
    merged_mesh.export(mesh_filename)

    ## add model to sdf
    model = ET.SubElement(world, 'model', name='random_spheres')
    static = ET.SubElement(model, 'static')
    static.text = 'true'
    link = ET.SubElement(model, 'link', name='base_link')

    visual = ET.SubElement(link, 'visual', name='merged_visual')
    visual_geom = ET.SubElement(visual, 'geometry')
    visual_mesh = ET.SubElement(visual_geom, 'mesh')
    visual_uri = ET.SubElement(visual_mesh, 'uri')
    # IGNITION GAZEBO FIX: Prepend standard file URI protocol
    visual_uri.text = f'file://{mesh_filename}'

    collision = ET.SubElement(link, 'collision', name='merged_collision')
    collision_geom = ET.SubElement(collision, 'geometry')
    collision_mesh = ET.SubElement(collision_geom, 'mesh')
    collision_uri = ET.SubElement(collision_mesh, 'uri')
    # IGNITION GAZEBO FIX: Prepend standard file URI protocol
    collision_uri.text = f'file://{mesh_filename}'

    ## update ground
    s_x = len(chunks_radius) * (poisson_domain_size + 1) + 4  # from -1m to 1m past the obstacles
    c_x = s_x / 2 - 1
    for model in world.findall('model'):
        if model.get('name') == 'ground_plane':
            field = model.find('link/collision/geometry/box/size')
            field.text = f'{s_x} {poisson_domain_size} 0.1'
            field = model.find('link/visual/geometry/box/size')
            field.text = f'{s_x} {poisson_domain_size} 0.1'
            field = model.find('link/pose')
            field.text = f'{c_x} 0 -0.05 0 0 0'
            ground = model
            break

    ## add ceiling
    ceil = copy.deepcopy(ground)
    ceil.set('name', 'ceil')
    field = ceil.find('link/pose')
    field.text = f'{c_x} 0 {poisson_domain_size + 0.05} 0 0 0'
    world.append(ceil)

    ## add left wall
    lwall = copy.deepcopy(ground)
    lwall.set('name', 'lwall')
    field = lwall.find('link/pose')
    field.text = f'{c_x} {poisson_domain_size / 2} {poisson_domain_size / 2} 1.57 0 0'
    world.append(lwall)

    ## add right wall
    rwall = copy.deepcopy(ground)
    rwall.set('name', 'rwall')
    field = rwall.find('link/pose')
    field.text = f'{c_x} {- poisson_domain_size / 2} {poisson_domain_size / 2} 1.57 0 0'
    world.append(rwall)

    ## write the updated SDF file to a new file
    tree.write(os.path.join(PATH_WORLDS, 'random3d.sdf'), xml_declaration=True, method='xml', encoding='UTF-8')

    print('randomization complete, new world file saved')