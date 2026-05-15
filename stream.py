import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def main():
    # 1. Konfigurasi Pipeline RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # 2. Konfigurasi Jendela Visualisasi 3D
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Pemetaan Permukaan 3D", width=800, height=600)
    
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.15, 0.15, 0.15]) 
    opt.point_size = 3.0 # Ukuran titik disesuaikan

    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    vis.add_geometry(axis_frame)
    
    pcd_vis = o3d.geometry.PointCloud()
    vis.add_geometry(pcd_vis)
    first_frame = True

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Konversi frame ke Point Cloud
            pc = rs.pointcloud()
            points = pc.calculate(depth_frame)
            v = points.get_vertices()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(verts)

            # 3. Passthrough Filter (Area pandang diperlebar menjadi 60x60 cm)
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-0.3, -0.3, 0.0), 
                max_bound=(0.3, 0.3, 1.0) # Kedalaman maksimal 1 meter
            )
            pcd_cropped = pcd.crop(bbox)

            # 4. Voxel Grid Filter (Kerapatan diubah menjadi 2 mm agar solid)
            pcd_down = pcd_cropped.voxel_down_sample(voxel_size=0.002)

            # Mewarnai titik berdasarkan Kedalaman (Sumbu Z)
            pts = np.asarray(pcd_down.points)
            if len(pts) > 0:
                z_vals = pts[:, 2] 
                z_min, z_max = z_vals.min(), z_vals.max()
                
                # Menghindari error pembagian dengan nol
                if z_max - z_min > 0:
                    z_norm = (z_vals - z_min) / (z_max - z_min) 
                else:
                    z_norm = np.zeros_like(z_vals)
                
                colors = np.zeros_like(pts)
                colors[:, 0] = 1.0 - z_norm 
                colors[:, 2] = z_norm       
                
                pcd_down.colors = o3d.utility.Vector3dVector(colors)

            # 5. Hitung Slope
            pcd_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20)
            )
            
            normals = np.asarray(pcd_down.normals)
            z_axis = np.array([0, 0, 1])
            
            if len(normals) > 0:
                dot_products = np.clip(np.abs(np.dot(normals, z_axis)), -1.0, 1.0)
                angles = np.arccos(dot_products)
                avg_slope = np.degrees(np.mean(angles))
                print(f"Jumlah Titik: {len(pcd_down.points)} | Kemiringan Rata-rata: {avg_slope:.2f} derajat", end="\r")

            # Update Jendela 3D
            pcd_vis.points = pcd_down.points
            pcd_vis.colors = pcd_down.colors
            
            # --- PERBAIKAN AUTO ZOOM ---
            # Tunggu sampai ada setidaknya 500 titik objek solid sebelum melakukan Auto-Zoom
            if first_frame and len(pcd_down.points) > 500:
                vis.reset_view_point(True)
                first_frame = False
                
            vis.update_geometry(pcd_vis)
            vis.poll_events()
            vis.update_renderer()

    finally:
        pipeline.stop()
        vis.destroy_window()

if __name__ == "__main__":
    main()