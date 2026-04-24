#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import trimesh
import numpy as np
import scipy as sp
import os
import copy
import argparse
import random

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PATH_WORLDS = os.path.join(BASE_DIR, 'worlds')
PATH_MODELS = os.path.join(PATH_WORLDS, 'meshes')

os.makedirs(PATH_WORLDS, exist_ok=True)
os.makedirs(PATH_MODELS, exist_ok=True)


## generate random pose values
def random_poses(n, radius, range, seed=None):
    poisson_engine = sp.stats.qmc.PoissonDisk(
        d=3,
        radius=radius / range,
        ncandidates=1000,
        seed=seed,
    )
    return (poisson_engine.random(n) - np.array([0.5, 0.5, 0])) * range


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate random 3D SDF worlds.')
    parser.add_argument('--seed', type=int, default=None, help='Random seed for reproducibility. Omit for a truly random map.')
    parser.add_argument('--domain-size', type=float, default=8.0, help='Size of each chunk')
    parser.add_argument('--chunks-radius', type=float, nargs='+', default=[2.5, 2.5, 2.5], help='Poisson radius for each chunk')
    parser.add_argument('--max-obstacles', type=int, default=1000, help='Maximum generated obstacles per chunk')
    parser.add_argument('--package-name', type=str, default='unified_autonomy_stack', help='ROS package name for URI (e.g. package://unified_autonomy_stack/...)')
    args = parser.parse_args()

    ## load the SDF file
    empty_sdf_path = os.path.join(PATH_WORLDS, 'empty.sdf')
    if os.path.exists(empty_sdf_path):
        tree = ET.parse(empty_sdf_path)
        root = tree.getroot()
    else:
        root = ET.Element('sdf', version='1.6')
        world_elem = ET.SubElement(root, 'world', name='default')
        ground_model = ET.SubElement(world_elem, 'model', name='ground_plane')
        ET.SubElement(ground_model, 'static').text = 'true'
        link = ET.SubElement(ground_model, 'link', name='link')
        ET.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        col = ET.SubElement(link, 'collision', name='collision')
        ET.SubElement(ET.SubElement(ET.SubElement(col, 'geometry'), 'box'), 'size').text = '1 1 0.1'
        vis = ET.SubElement(link, 'visual', name='visual')
        ET.SubElement(ET.SubElement(ET.SubElement(vis, 'geometry'), 'box'), 'size').text = '1 1 0.1'
        tree = ET.ElementTree(root)

    ## find the <world> element
    world = root.find('world')
    if world is None:
        raise ValueError('no <world> element found in the SDF file')

    ## add random obstacles
    # `chunks_radius` controls the density. A lower value means obstacles are placed closer 
    # to each other (higher density). The array length designates how many sections/blocks are made.
    # Example for variable density across 3 blocks: chunks_radius = [1.0, 1.5, 2.0]
    chunks_radius = args.chunks_radius
    seed = args.seed if args.seed is not None else 0
    poisson_domain_size = args.domain_size
    n_obstacles_max = args.max_obstacles
    param_str = f"s{seed}_d{int(poisson_domain_size)}_r{'-'.join(str(r) for r in chunks_radius)}"
    x_offset = 2 + poisson_domain_size / 2
    meshes = []
    base_mesh = trimesh.creation.icosphere(subdivisions=2, radius=0.5)
    for j,r in enumerate(chunks_radius):
        current_seed = seed + j
        poisson_obs = random_poses(n_obstacles_max, r, poisson_domain_size, current_seed)
        n_obstacles = poisson_obs.shape[0]
        print(f'placing {n_obstacles} obstacles in the chunk {j} (Poisson radius: {r})')


        for pos in poisson_obs:
            mesh = base_mesh.copy()
            mesh.apply_translation(pos + [x_offset, 0, 0])
            meshes.append(mesh)

        x_offset += poisson_domain_size + r

    ## export dae
    mesh_filename = os.path.join(PATH_MODELS, f'random_spheres_{param_str}.dae')
    merged_mesh = trimesh.util.concatenate(meshes)
    merged_mesh.export(mesh_filename)

    ## add model to sdf
    model = ET.SubElement(world, 'model', name=f'random_spheres_{param_str}')
    static = ET.SubElement(model, 'static')
    static.text = 'true'
    link = ET.SubElement(model, 'link', name='base_link')

    visual = ET.SubElement(link, 'visual', name='merged_visual')
    visual_geom = ET.SubElement(visual, 'geometry')
    visual_mesh = ET.SubElement(visual_geom, 'mesh')
    visual_uri = ET.SubElement(visual_mesh, 'uri')
    
    # Format URI to use package name
    mesh_basename = os.path.basename(mesh_filename)
    package_uri = f'package://{args.package_name}/worlds/meshes/{mesh_basename}'
    
    visual_uri.text = package_uri

    collision = ET.SubElement(link, 'collision', name='merged_collision')
    collision_geom = ET.SubElement(collision, 'geometry')
    collision_mesh = ET.SubElement(collision_geom, 'mesh')
    collision_uri = ET.SubElement(collision_mesh, 'uri')
    collision_uri.text = package_uri

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
    tree.write(os.path.join(PATH_WORLDS, f'random3d_{param_str}.sdf'), xml_declaration=True, method='xml', encoding='UTF-8')

    print('randomization complete, new world file saved')