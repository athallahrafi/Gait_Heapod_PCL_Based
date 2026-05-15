import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def main():
    # 1. Konfigurasi Pipeline RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Resolusi optimal
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # --- PERBAIKAN: Konfigurasi Jendela Visualisasi 3D ---
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Pemetaan Permukaan 3D", width=800, height=600)
    
    # Mengatur render (Background abu gelap, titik diperbesar)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.15, 0.15, 0.15]) 
    opt.point_size = 5.0 

    # Menambahkan garis sumbu bantu (Size 10cm)
    # Merah=Kanan/Kiri(X), Hijau=Atas/Bawah(Y), Biru=Maju/Mundur(Z)
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    vis.add_geometry(axis_frame)
    
    pcd_vis = o3d.geometry.PointCloud()
    vis.add_geometry(pcd_vis)
    first_frame = True
    # -----------------------------------------------------

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Ekstrak data 3D
            pc = rs.pointcloud()
            points = pc.calculate(depth_frame)
            v = points.get_vertices()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(verts)

            # 3. Passthrough Filter (Crop 40x40 cm)
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-0.2, -0.2, 0), 
                max_bound=(0.2, 0.2, 2.0)
            )
            pcd_cropped = pcd.crop(bbox)

            # 4. Voxel Grid Filter (1cm = 0.01m)
            pcd_down = pcd_cropped.voxel_down_sample(voxel_size=0.01)

            # --- PERBAIKAN: Mewarnai titik menjadi Hijau agar jelas ---
            # --- PERBAIKAN BARU: Mewarnai titik berdasarkan Kedalaman (Sumbu Z) ---
            pts = np.asarray(pcd_down.points)
            if len(pts) > 0:
                z_vals = pts[:, 2] # Mengambil nilai Z (kedalaman) dari semua titik
                z_min, z_max = z_vals.min(), z_vals.max()
                
                # Normalisasi nilai Z menjadi rentang 0.0 hingga 1.0
                z_norm = (z_vals - z_min) / (z_max - z_min + 1e-6) 
                
                # Membuat array warna kosong (RGB)
                colors = np.zeros_like(pts)
                colors[:, 0] = 1.0 - z_norm # Merah (Kuat saat dekat, melemah saat jauh)
                colors[:, 2] = z_norm       # Biru (Melemah saat dekat, kuat saat jauh)
                
                # Terapkan warna ke Point Cloud
                pcd_down.colors = o3d.utility.Vector3dVector(colors)

            # 5. Hitung Slope
            pcd_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
            )
            
            normals = np.asarray(pcd_down.normals)
            z_axis = np.array([0, 0, 1])
            
            if len(normals) > 0:
                dot_products = np.clip(np.abs(np.dot(normals, z_axis)), -1.0, 1.0)
                angles = np.arccos(dot_products)
                avg_slope = np.degrees(np.mean(angles))
                print(f"Jumlah Titik: {len(pcd_down.points)} | Kemiringan Rata-rata: {avg_slope:.2f} derajat")

            # Update Jendela 3D
            pcd_vis.points = pcd_down.points
            pcd_vis.colors = pcd_down.colors
            
            if first_frame and len(pcd_down.points) > 0:
                # Memaksa kamera Open3D untuk melihat lurus ke arah objek
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