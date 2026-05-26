import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def main():
    # 1. Konfigurasi Pipeline RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    # D405 optimal pada resolusi tinggi untuk depth
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Memulai streaming
    pipeline.start(config)

    try:
        while True:
            # Menunggu frame
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Konversi frame ke Point Cloud
            pc = rs.pointcloud()
            points = pc.calculate(depth_frame)
            v = points.get_vertices()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3) # xyz

            # 2. Inisialisasi Open3D Point Cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(verts)

            # 3. Passthrough Filter (Crop area 40cm x 40cm)
            # Mengambil x: -0.2 ke 0.2 dan y: -0.2 ke 0.2
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=(-0.2, -0.2, 0), 
                max_bound=(0.2, 0.2, 2.0)
            )
            pcd_cropped = pcd.crop(bbox)

            # 4. Voxel Grid Filter (1cm = 0.01m)
            pcd_down = pcd_cropped.voxel_down_sample(voxel_size=0.01)

            # 5. Estimasi Vektor Normal (untuk hitung Slope)
            pcd_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
            )

            # Hitung kemiringan (Slope) setiap titik
            # Normal [nx, ny, nz]. Sudut kemiringan terhadap sumbu Z
            normals = np.asarray(pcd_down.normals)
            z_axis = np.array([0, 0, 1])
            
            # Menghitung sudut antara normal dan sumbu vertikal
            angles = np.arccos(np.abs(np.dot(normals, z_axis)))
            avg_slope = np.degrees(np.mean(angles))

            print(f"Jumlah Titik: {len(pcd_down.points)} | Rata-rata Slope: {avg_slope:.2f} derajat")

    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()