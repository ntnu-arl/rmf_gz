import os
import random
import trimesh
import numpy as np
import xml.etree.ElementTree as ET

class URDFEnvironmentBaker:
    def __init__(self, base_path):
        self.base_path = base_path
        self.area_length = 50.0  
        self.area_width = 7.0
        self.max_height = 6.0
        
    def load_geometry_from_urdf(self, urdf_path):
        """Parses a URDF and returns a trimesh object, handling meshes and primitives."""
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # 1. Look for geometry in visual tags first. If missing, look in collision tags.
            geometries = root.findall('.//visual/geometry')
            if not geometries:
                geometries = root.findall('.//collision/geometry')
                
            for geom in geometries:
                
                # External Mesh
                mesh_tag = geom.find('mesh')
                if mesh_tag is not None:
                    filename = mesh_tag.get('filename')
                    if filename:
                        clean_path = filename.replace('package://', '', 1).split('/', 1)[-1]
                        clean_path = clean_path.replace('file://', '', 1)
                        
                        primary_path = os.path.join(self.base_path, "..", clean_path)
                        fallback_path = os.path.join(os.path.dirname(urdf_path), clean_path)
                        
                        target_path = None
                        if os.path.exists(primary_path): target_path = primary_path
                        elif os.path.exists(fallback_path): target_path = fallback_path
                        
                        if target_path:
                            mesh = trimesh.load(target_path)
                            if isinstance(mesh, trimesh.Scene):
                                mesh = mesh.dump(concatenate=True)
                                
                            # Handle mesh scaling if it exists in the URDF
                            scale_str = mesh_tag.get('scale')
                            if scale_str:
                                scale_vals = [float(s) for s in scale_str.split()]
                                scale_matrix = np.eye(4)
                                scale_matrix[0, 0] = scale_vals[0]
                                scale_matrix[1, 1] = scale_vals[1]
                                scale_matrix[2, 2] = scale_vals[2]
                                mesh.apply_transform(scale_matrix)
                                
                            return mesh
                        else:
                            print(f"Warning: Missing external mesh file for: {urdf_path}")
                            return None

                # Primitive Box
                box_tag = geom.find('box')
                if box_tag is not None:
                    size = [float(x) for x in box_tag.get('size', '1 1 1').split()]
                    return trimesh.creation.box(extents=size)

                # Primitive Cylinder
                cyl_tag = geom.find('cylinder')
                if cyl_tag is not None:
                    radius = float(cyl_tag.get('radius', '0.5'))
                    length = float(cyl_tag.get('length', '1.0'))
                    return trimesh.creation.cylinder(radius=radius, height=length)

                # Primitive Sphere
                sph_tag = geom.find('sphere')
                if sph_tag is not None:
                    radius = float(sph_tag.get('radius', '0.5'))
                    return trimesh.creation.icosphere(radius=radius)

            print(f"Warning: No valid geometry found in {urdf_path}")
            
        except Exception as e:
            print(f"Could not parse URDF {urdf_path}: {e}")
            
        return None

    def generate_random_pose(self):
        """Generates a random 4x4 transformation matrix."""
        x = random.uniform(5, self.area_length)
        y = random.uniform(-self.area_width/2, self.area_width/2)
        z = random.uniform(0.2, self.max_height)
        
        translation = [x, y, z]
        rotation = trimesh.transformations.random_rotation_matrix()
        
        pose = rotation
        pose[:3, 3] = translation
        return pose

    def bake(self, num_obstacles=45, output_name="random_baked_env.dae"):
        all_meshes = []
        
        folders = ['objects_large', 'objects_medium', 'panels', 'pillars', 'thin']
        urdf_files = []
        
        for f in folders:
            folder_path = os.path.join(self.base_path, f)
            if os.path.exists(folder_path):
                files = [os.path.join(folder_path, s) for s in os.listdir(folder_path) if s.endswith('.urdf')]
                urdf_files.extend(files)

        if not urdf_files:
            print(f"No URDF files found in {self.base_path}")
            return

        print(f"Found {len(urdf_files)} unique obstacle types. Baking {num_obstacles} instances...")
        for file in urdf_files:
            print(f" - {file}")

        for i in range(num_obstacles):
            selected_urdf = random.choice(urdf_files)
            mesh = self.load_geometry_from_urdf(selected_urdf)
            
            if mesh:
                mesh.apply_transform(self.generate_random_pose())
                all_meshes.append(mesh)

        if all_meshes:
            combined = trimesh.util.concatenate(all_meshes)
            output_path_dae = os.path.join(self.base_path, output_name)
            output_path_obj = os.path.join(self.base_path, output_name.replace('.dae', '.obj'))
            
            # Export with fallback protection
            try:
                combined.export(output_path_dae)
                print(f"Bake complete! Combined {len(all_meshes)} meshes. Output saved to: {output_path_dae}")
            except Exception as e:
                print(f"\n[!] Could not export .dae file. You are likely missing the 'pycollada' library.")
                print(f"    Error details: {e}")
                print(f"    Falling back to .obj format...")
                combined.export(output_path_obj)
                print(f"Bake complete! Combined {len(all_meshes)} meshes. Output saved to: {output_path_obj}")
        else:
            print("Bake failed: No geometries could be successfully generated or loaded.")

if __name__ == "__main__":
    PATH_TO_ASSETS = os.path.expanduser("/home/marvin/lmf_ws/src/lmf_sim/resources/environment_assets")
    baker = URDFEnvironmentBaker(PATH_TO_ASSETS)
    baker.bake(num_obstacles=200, output_name="random_objects.dae")