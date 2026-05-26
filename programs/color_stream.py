import pyrealsense2 as rs
import numpy as np
import open3d as o3d

def main():
    # 1. Konfigurasi Pipeline RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Mengaktifkan aliran Kedalaman (Depth) dan Warna (Color)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Memulai pipeline
    profile = pipeline.start(config)

    # Alat untuk menyelaraskan gambar 3D dengan gambar Warna
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 2. Konfigurasi Jendela Visualisasi 3D
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="RealSense RGB-D Scan Penuh", width=800, height=600)
    
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.05, 0.05, 0.05]) # Hitam gelap
    opt.point_size = 2.0 # Ukuran titik lebih kecil agar terlihat rapat
    
    pcd_vis = o3d.geometry.PointCloud()
    vis.add_geometry(pcd_vis)
    first_frame = True

    try:
        while True:
            # Menunggu dan menyelaraskan frame
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            # 3. Konversi ke Format Open3D
            depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
            color_image = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))
            
            # Menggabungkan warna asli dengan kedalaman 3D
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image, depth_image, depth_scale=1000.0, depth_trunc=5.0, convert_rgb_to_intensity=False)

            # Mendapatkan parameter intrinsik lensa D405
            intr = color_frame.profile.as_video_stream_profile().intrinsics
            pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

            # Membuat Point Cloud
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, pinhole_camera_intrinsic)
            
            # Membalik orientasi agar tidak terbalik (Standar Open3D)
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

            # 4. Filter ringan untuk membersihkan noise (tanpa crop yang ekstrem)
            pcd_down = pcd.voxel_down_sample(voxel_size=0.002) # Kerapatan 2 mm!

            # Update Jendela 3D
            pcd_vis.points = pcd_down.points
            pcd_vis.colors = pcd_down.colors
            
            if first_frame and len(pcd_down.points) > 0:
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