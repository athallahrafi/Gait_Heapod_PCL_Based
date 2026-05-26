import time
import cv2
import numpy as np
import pyrealsense2 as rs

# =====================================================================
# 1. KONFIGURASI AWAL HARDWARE & STREAM REALSENSE D405
# =====================================================================
pipeline = rs.pipeline()
config = rs.config()

# Mengaktifkan stream Depth dan Color (Resolusi standar 640x480 pada 30 FPS)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Memulai pembacaan sensor
profile = pipeline.start(config)

# Mengambil depth scale dari sensor (Untuk konversi nilai mentah ke satuan meter/mm)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Objek untuk menyelaraskan (align) matriks depth agar presisi dengan kamera RGB
align_to = rs.stream.color
align = rs.align(align_to)

# =====================================================================
# 2. KONFIGURASI PARAMETER FILTER RESEARCH (METOPEN)
# =====================================================================
# Batasan jarak deteksi (dalam milimeter): 100mm (10cm) hingga 500mm (50cm)
# Sesuai misi: menganalisa track pada rentang 40cm ke depan.
DIST_MIN_MM = 100 
DIST_MAX_MM = 500

# Threshold toleransi kedalaman lantai aman (dalam milimeter)
# Jika jarak lantai ke kamera mendadak turun/lebih dalam dari nilai ini, dianggap "jurang"
FLOOR_THRESHOLD_MM = 450 

# Inisialisasi variabel untuk perhitungan FPS
prev_frame_time = 0

print("=== Program Utama Intel RealSense D405 untuk KRSRI Berhasil Dijalankan ===")
print("Tekan tombol 'q' pada jendela grafis untuk keluar.")

try:
    while True:
        # =====================================================================
        # 3. AKUISISI DATA & ALIGNMENT FRAME
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
        
        # Konversi matriks depth mentah langsung ke satuan milimeter riil
        depth_mm = depth_raw * depth_scale * 1000
        
        # =====================================================================
        # 4. IMPLEMENTASI FILTERING DATA (PROSES PCL/DEPTH)
        # =====================================================================
        # Filter Spasial Jarak: Ambil data yang murni berada di rentang area 40cm target
        mask_jarak = (depth_mm > DIST_MIN_MM) & (depth_mm < DIST_MAX_MM)
        filtered_depth = np.where(mask_jarak, depth_mm, 0)
        
        # Segmentasi Wilayah ROI Vertikal (Membagi lebar layar 640 menjadi 3 Zona)
        # Sisi Kiri: 0 - 213, Sisi Tengah: 214 - 426, Sisi Kanan: 427 - 640
        lebar_frame = color_image.shape[1]
        garis_kiri = lebar_frame // 3
        garis_kanan = (lebar_frame // 3) * 2
        
        # Memotong matriks depth berdasarkan wilayah ROI pada baris tertentu (misal area deteksi roda depan)
        # Kita ambil sampel baris piksel 300 hingga 400 (area bawah kamera/lantai terdekat)
        baris_start, baris_end = 300, 400
        
        roi_kiri = filtered_depth[baris_start:baris_end, :garis_kiri]
        roi_tengah = filtered_depth[baris_start:baris_end, garis_kiri:garis_kanan]
        roi_kanan = filtered_depth[baris_start:baris_end, garis_kanan:]
        
        # =====================================================================
        # 5. LOGIKA EVALUASI PERMUKAAN TRACK (NAVIGASI & MITIGASI JURANG)
        # =====================================================================
        # Hitung rata-rata jarak di tiap ROI (abaikan nilai 0 hasil filter)
        mean_kiri = np.mean(roi_kiri[roi_kiri > 0]) if np.any(roi_kiri > 0) else 0
        mean_tengah = np.mean(roi_tengah[roi_tengah > 0]) if np.any(roi_tengah > 0) else 0
        mean_kanan = np.mean(roi_kanan[roi_kanan > 0]) if np.any(roi_kanan > 0) else 0
        
        # Logika Deteksi Tepi (Garis Warna)
        warna_kiri = (0, 255, 0)   # Default Hijau (Aman)
        warna_kanan = (0, 255, 0)  # Default Hijau (Aman)
        status_navigasi = "ROBOT AMAN: JALUR TENGAH"
        
        # Cek kondisi tepi terperosok (Jika rata-rata kedalaman melebihi batas atau 0/ruang kosong)
        if mean_kiri > FLOOR_THRESHOLD_MM or mean_kiri == 0:
            warna_kiri = (0, 0, 255) # Berubah Merah (Bahaya/Jurang Kiri)
            status_navigasi = "PERINGATAN: KOREKSI YAW KE KANAN!"
            
        if mean_kanan > FLOOR_THRESHOLD_MM or mean_kanan == 0:
            warna_kanan = (0, 0, 255) # Berubah Merah (Bahaya/Jurang Kanan)
            status_navigasi = "PERINGATAN: KOREKSI YAW KE KIRI!"
            
        if (mean_kiri > FLOOR_THRESHOLD_MM or mean_kiri == 0) and (mean_kanan > FLOOR_THRESHOLD_MM or mean_kanan == 0):
            status_navigasi = "JALUR SEMPIT: KUNCI HEADING YAW (IMU/LURUS)"

        # =====================================================================
        # 6. VISUALISASI GRAFIS, GARIS ROI, DAN PENGHITUNG FPS
        # =====================================================================
        # Menghitung FPS
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps_text = f"FPS: {int(fps)}"
        
        # Menggambar Garis ROI Vertikal pembagi wilayah pada Raw Kamera
        cv2.line(color_image, (garis_kiri, 0), (garis_kiri, 480), warna_kiri, 2)
        cv2.line(color_image, (garis_kanan, 0), (garis_kanan, 480), warna_kanan, 2)
        
        # Menggambar Kotak Area Sampel Deteksi Evaluasi Kedalaman (Baris 300 s.d 400)
        cv2.rectangle(color_image, (0, baris_start), (lebar_frame, baris_end), (255, 255, 0), 1)
        
        # Menampilkan teks status kontrol navigasi hasil analisa data depth
        cv2.putText(color_image, status_navigasi, (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Menampilkan nilai FPS di pojok kanan atas frame
        cv2.putText(color_image, fps_text, (520, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Tampilkan Window Gabungan RAW Kamera + Garis Tepi Analisis Depth PCL
        cv2.imshow("Combined RAW + PCL Edge View (KRSRI Research)", color_image)
        
        # Logika tombol keluar program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    # Mematikan pipeline kamera saat aplikasi ditutup
    pipeline.stop()
    cv2.destroyAllWindows()
    print("=== Program Dihentikan dengan Aman ===")