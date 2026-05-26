import time
import cv2
import numpy as np
import pyrealsense2 as rs

# 1. Konfigurasi Stream RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# Objek untuk menyelaraskan depth ke color stream
align_to = rs.stream.color
align = rs.align(align_to)

# Inisialisasi variabel untuk menghitung FPS
prev_frame_time = 0
new_frame_time = 0

try:
    while True:
        # 2. Ambil Frame
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
            
        # Ubah frame menjadi numpy array agar bisa diproses OpenCV
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # 3. KALKULASI FPS
        new_frame_time = time.time()
        # Rumus: 1 / selisih waktu (dalam detik)
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        
        # Mengubah nilai FPS menjadi string tanpa desimal
        fps_text = f"FPS: {int(fps)}"
        
        # 4. PROSES DETEKSI TEPI LINTASAN (ROI) BERDASARKAN DEPTH
        # (Tempatkan logika pengolahan PCL kamu di sini)
        
        # Contoh simulasi garis tepi (Ganti dengan koordinat asli PCL-mu)
        cv2.line(color_image, (150, 0), (150, 480), (0, 0, 255), 2) 
        cv2.line(color_image, (490, 0), (490, 480), (0, 0, 255), 2) 
        
        # 5. TAMPILKAN FPS DAN INDIKATOR PADA FRAME RGB
        # Menampilkan status teks ROI
        cv2.putText(color_image, "ROI: Edge Detection Active", (30, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Menampilkan nilai FPS di pojok kanan atas (Warna kuning agar kontras)
        cv2.putText(color_image, fps_text, (500, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        
        # 6. TAMPILKAN HASIL GABUNGAN
        cv2.imshow("Combined RAW + PCL Edge View", color_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()