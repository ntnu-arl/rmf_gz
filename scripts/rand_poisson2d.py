#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import trimesh
import numpy as np
import scipy as sp
import os
import copy


PATH_RESOURCES = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../resources')
PATH_WORLDS = os.path.join(PATH_RESOURCES, 'worlds/')
PATH_MODELS = os.path.join(PATH_RESOURCES, 'environment_assets/')


## generate random pose values
def random_poses(n, radius, range, seed=None):
    poisson_engine = sp.stats.qmc.PoissonDisk(
        d=2,
        radius=radius / range,
        ncandidates=500,
        seed=seed,
    )
    return (poisson_engine.random(n) - np.array([0.5, 0.5])) * range


if __name__ == '__main__':
    ## load the SDF file
    tree = ET.parse(os.path.join(PATH_WORLDS, 'empty.sdf'))
    root = tree.getroot()

    ## find the <world> element
    world = root.find('world')
    if world is None:
        raise ValueError('no <world> element found in the SDF file')

    ## add random obstacles
    chunks_radius = [2] * 2
    seed = 200
    poisson_domain_size = 4
    x_offset = 2 + poisson_domain_size / 2
    meshes = []
    base_mesh = trimesh.creation.cylinder(radius=0.5, height=poisson_domain_size)
    for j,r in enumerate(chunks_radius):
        poisson_obs = random_poses(1000, r, poisson_domain_size, seed+j)
        n_obstacles = poisson_obs.shape[0]
        print(f'placing {n_obstacles} obstacles in the chunk {j} (Poisson radius: {r})')

        for pos in poisson_obs:
            mesh = base_mesh.copy()
            mesh.apply_translation([pos[0] + x_offset, pos[1], poisson_domain_size / 2])
            meshes.append(mesh)

        x_offset += poisson_domain_size + 1

    ## export dae
    mesh_filename = os.path.join(PATH_MODELS, 'random_pillars.dae')
    merged_mesh = trimesh.util.concatenate(meshes)
    merged_mesh.export(mesh_filename)

    ## add model to sdf
    model = ET.SubElement(world, 'model', name='random_pillars')
    static = ET.SubElement(model, 'static')
    static.text = 'true'
    link = ET.SubElement(model, 'link', name='base_link')

    visual = ET.SubElement(link, 'visual', name='merged_visual')
    visual_geom = ET.SubElement(visual, 'geometry')
    visual_mesh = ET.SubElement(visual_geom, 'mesh')
    visual_uri = ET.SubElement(visual_mesh, 'uri')
    visual_uri.text = f'{mesh_filename}'

    collision = ET.SubElement(link, 'collision', name='merged_collision')
    collision_geom = ET.SubElement(collision, 'geometry')
    collision_mesh = ET.SubElement(collision_geom, 'mesh')
    collision_uri = ET.SubElement(collision_mesh, 'uri')
    collision_uri.text = f'{mesh_filename}'

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
    tree.write(os.path.join(PATH_WORLDS, 'random2d.sdf'), xml_declaration=True, method='xml', encoding='UTF-8')

    print('randomization complete, new world file saved')
