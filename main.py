import open3d as o3d
import numpy as np

def original_test():
    # bunny = o3d.data.BunnyMesh()
    # mesh  = o3d.io.read_triangle_mesh(bunny.path)
    pcd = o3d.io.read_point_cloud("data.pcd")
    pcd.estimate_normals()

    # estimate radius for rolling ball
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist   

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd,
            o3d.utility.DoubleVector([radius, radius * 2]))
    
    o3d.visualization.draw_geometries([pcd])
    alpha = 0.03
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

def ball_rec():
    # gt_mesh = o3dtut.get_bunny_mesh()
    # gt_mesh.compute_vertex_normals()
    # pcd = gt_mesh.sample_points_poisson_disk(3000)
    pcd = o3d.io.read_point_cloud("data.pcd")
    pcd.estimate_normals()
    o3d.visualization.draw_geometries([pcd])
    radii = [0.005, 0.01, 0.02, 0.04]
    rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))
    
    o3d.visualization.draw_geometries([pcd, rec_mesh])

def poisson_rec():
    pcd = o3d.io.read_point_cloud("data.pcd")
    #pcd.estimate_normals()
    print(pcd)

    # Get normals
    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals

    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(360)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)
    
    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9)
    print(mesh)
    o3d.visualization.draw_geometries([mesh],
                                    zoom=0.664,
                                    front=[-0.4761, -0.4698, -0.7434],
                                    lookat=[1.8900, 3.2596, 0.9284],
                                    up=[0.2304, -0.8825, 0.4101])

def main():
    poisson_rec()

if __name__ == '__main__':
    main()