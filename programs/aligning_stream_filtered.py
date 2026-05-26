import time
import cv2
import numpy as np
import pyrealsense2 as rs

# =====================================================================
# 1. KONFIGURASI HARDWARE & STREAM REALSENSE D405
# =====================================================================
pipeline = rs.pipeline()
config = rs.config()

# Mengaktifkan stream Depth dan Color (Resolusi 640x480 pada 30 FPS)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Memulai pembacaan sensor
profile = pipeline.start(config)

# Mengambil depth scale dari sensor untuk konversi ke satuan mm
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Objek untuk menyelaraskan (align) matriks depth agar presisi dengan kamera RGB
align_to = rs.stream.color
align = rs.align(align_to)

# =====================================================================
# 2. PARAMETER KONTROL & FILTER (RESEARCH METOPEN)
# =====================================================================
# Jarak deteksi sesuai misi: kisaran 40cm ke depan (100mm - 500mm)
DIST_MIN_MM = 100 
DIST_MAX_MM = 500

# Batas toleransi kedalaman lantai aman (dalam milimeter)
# Jika jarak lantai ke kamera mendadak lebih dalam dari nilai ini, dianggap "jurang"
FLOOR_THRESHOLD_MM = 450 

# Inisialisasi variabel waktu untuk perhitungan FPS
prev_frame_time = 0

print("=== Program Utama Intel RealSense D405 untuk KRSRI Berhasil Dijalankan ===")
print("Tekan tombol 'q' pada jendela grafis untuk keluar.")

try:
    while True:
        # =====================================================================
        # 3. AKUISISI DATA & SINKRONISASI FRAME
        # =====================================================================
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
            
        # Konversi data frame mentah menjadi array numpy (Matriks)
        depth_raw = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Dapatkan dimensi frame (Lebar: 640, Tinggi: 480)
        tinggi_frame, lebar_frame, _ = color_image.shape
        
        # Konversi matriks depth mentah langsung ke satuan milimeter riil
        depth_mm = depth_raw * depth_scale * 1000
        
        # =====================================================================
        # 4. PROSES FILTERING DATA DEPTH
        # =====================================================================
        # Filter Jarak: Hanya ambil data di rentang target 40cm
        mask_jarak = (depth_mm > DIST_MIN_MM) & (depth_mm < DIST_MAX_MM)
        filtered_depth = np.where(mask_jarak, depth_mm, 0)
        
        # =====================================================================
        # 5. SCANNING DINAMIS UNTUK MENCARI TEPI LINTASAN (OUTPUT JURANG KANAN-KIRI)
        # =====================================================================
        # Tentukan baris piksel sampel sebagai area "Virtual Bumper" (misal baris 350)
        baris_sampel = 350
        
        # Batas awal default jika tepi tidak ditemukan (di ujung layar)
        tepi_kiri_x = 0
        tepi_kanan_x = lebar_frame - 1
        
        # Scan dari TENGAH LAYAR ke ARAH KIRI untuk mencari tepi kiri lintasan
        for x in range(lebar_frame // 2, 0, -1):
            jarak_piksel = filtered_depth[baris_sampel, x]
            # Jika jarak terdeteksi sebagai jurang/patahan lantai atau kosong (0)
            if jarak_piksel > FLOOR_THRESHOLD_MM or jarak_piksel == 0:
                tepi_kiri_x = x
                break
                
        # Scan dari TENGAH LAYAR ke ARAH KANAN untuk mencari tepi kanan lintasan
        for x in range(lebar_frame // 2, lebar_frame):
            jarak_piksel = filtered_depth[baris_sampel, x]
            if jarak_piksel > FLOOR_THRESHOLD_MM or jarak_piksel == 0:
                tepi_kanan_x = x
                break

        # Hitung koordinat tengah lintasan riil berdasarkan deteksi tepi
        center_track_x = (tepi_kiri_x + tepi_kanan_x) // 2
        center_kamera_x = lebar_frame // 2
        
        # Hitung Nilai Error Yaw (Pusat Kamera vs Pusat Lintasan Aktual)
        error_yaw = center_kamera_x - center_track_x

        # Tentukan status instruksi navigasi berdasarkan nilai error
        if error_yaw > 20:
            status_navigasi = f"KOREKSI: YAW KE KIRI (Err: {error_yaw})"
        elif error_yaw < -20:
            status_navigasi = f"KOREKSI: YAW KE KANAN (Err: {error_yaw})"
        else:
            status_navigasi = "ROBOT AMAN: JALUR TENGAH"

        # =====================================================================
        # 6. KALKULASI FPS & VISUALISASI GRAFIS DINAMIS
        # =====================================================================
        # Menghitung nilai FPS secara real-time (Ditempatkan SEBELUM putText)
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps_text = f"FPS: {int(fps)}"
        
        # 1. Menggambar GARIS TEPI KIRI ASLI (Warna Merah)
        cv2.line(color_image, (tepi_kiri_x, 0), (tepi_kiri_x, tinggi_frame), (0, 0, 255), 2)
        
        # 2. Menggambar GARIS TEPI KANAN ASLI (Warna Merah)
        cv2.line(color_image, (tepi_kanan_x, 0), (tepi_kanan_x, tinggi_frame), (0, 0, 255), 2)
        
        # 3. Menggambar TITIK TENGAH LINTASAN AKTUAL HASIL DETEKS_ PCL (Hijau)
        cv2.circle(color_image, (center_track_x, baris_sampel), 8, (0, 255, 0), -1)
        
        # 4. Menggambar TITIK ACUAN TENGAH KAMERA (Putih)
        cv2.circle(color_image, (center_kamera_x, baris_sampel), 5, (255, 255, 255), -1)
        
        # 5. Menggambar garis bantu horizontal area sensor virtual (Cyan)
        cv2.line(color_image, (0, baris_sampel), (lebar_frame, baris_sampel), (255, 255, 0), 1)
        
        # 6. Menampilkan teks informasi status kontrol navigasi dan FPS ke layar
        cv2.putText(color_image, status_navigasi, (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(color_image, fps_text, (520, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Tampilkan Window Hasil Overlay Gambar Kamera + Analisis Tepi PCL
        cv2.imshow("Combined RAW + PCL Edge View (KRSRI Research)", color_image)
        
        # Tombol interupsi keluar program ('q')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    # Memastikan pipeline hardware ditutup dengan bersih saat program stop
    pipeline.stop()
    cv2.destroyAllWindows()
    print("=== Program Dihentikan dengan Aman ===")