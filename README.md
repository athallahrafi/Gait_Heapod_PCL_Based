# 🦾 Gait Hexapod PCL-Based

> Sistem analisis dan generasi pola gaya berjalan (gait) untuk robot hexapod berbasis Point Cloud Library (PCL)

![C++](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat-square&logo=c%2B%2B)
![PCL](https://img.shields.io/badge/PCL-1.x-orange?style=flat-square)
![ROS](https://img.shields.io/badge/ROS-Noetic%2FMelodic-green?style=flat-square&logo=ros)
![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)

---

## 📖 Deskripsi

Proyek ini mengimplementasikan sistem analisis dan perencanaan gait (pola gerak kaki) untuk robot **hexapod** (berkaki enam) dengan memanfaatkan data **point cloud 3D** menggunakan **Point Cloud Library (PCL)**. Sistem ini memungkinkan robot hexapod untuk memahami lingkungan sekitarnya secara tiga dimensi, lalu merencanakan gerakan kaki yang stabil dan adaptif berdasarkan data terrain yang diperoleh.

Pendekatan berbasis PCL memungkinkan pemrosesan real-time dari sensor depth (seperti LiDAR atau RGB-D) untuk:
- Deteksi dan segmentasi permukaan terrain
- Perencanaan titik pijak (foothold planning) yang optimal
- Generasi pola gait yang adaptif terhadap medan

---

## ✨ Fitur Utama

- **Point Cloud Processing** — Pemrosesan data 3D dari sensor depth secara real-time
- **Terrain Analysis** — Analisis topografi permukaan untuk mendeteksi rintangan dan medan tidak rata
- **Foothold Planning** — Perencanaan titik pijak optimal untuk setiap kaki hexapod
- **Gait Generation** — Generasi pola gerak kaki (tripod, wave, ripple gait) yang adaptif
- **Visualisasi 3D** — Visualisasi point cloud dan trajektori kaki secara real-time
- **Integrasi ROS** — Mendukung integrasi dengan Robot Operating System (ROS)

---

## 🛠️ Prasyarat

Pastikan sistem Anda memiliki dependensi berikut terinstal:

| Dependensi | Versi Minimum | Catatan |
|---|---|---|
| Ubuntu | 18.04 / 20.04 | Disarankan Ubuntu 20.04 LTS |
| CMake | ≥ 3.10 | Build system |
| PCL (Point Cloud Library) | ≥ 1.10 | Library utama |
| Eigen3 | ≥ 3.3 | Komputasi linear algebra |
| VTK | ≥ 7.0 | Visualisasi 3D |
| ROS | Melodic / Noetic | Opsional, untuk integrasi robot |
| OpenCV | ≥ 4.0 | Opsional, untuk pemrosesan citra |

---

## 🚀 Instalasi

### 1. Clone Repositori

```bash
git clone https://github.com/athallahrafi/Gait_Heapod_PCL_Based.git
cd Gait_Heapod_PCL_Based
```

### 2. Install Dependensi PCL

```bash
sudo apt-get update
sudo apt-get install -y libpcl-dev pcl-tools
sudo apt-get install -y libeigen3-dev libvtk7-dev
```

### 3. Build Proyek

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 4. (Opsional) Instalasi dengan ROS

Jika menggunakan ROS, letakkan paket ini di dalam workspace catkin Anda:

```bash
cd ~/catkin_ws/src
git clone https://github.com/athallahrafi/Gait_Heapod_PCL_Based.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 📁 Struktur Proyek

```
Gait_Heapod_PCL_Based/
├── src/                    # Source code utama
│   ├── gait_planner.cpp    # Modul perencanaan gait
│   ├── pcl_processor.cpp   # Pemrosesan point cloud
│   ├── terrain_analyzer.cpp# Analisis terrain
│   └── foothold_planner.cpp# Perencanaan titik pijak
├── include/                # Header files
│   ├── gait_planner.h
│   ├── pcl_processor.h
│   └── terrain_analyzer.h
├── config/                 # File konfigurasi
│   └── params.yaml
├── launch/                 # ROS launch files
│   └── hexapod_gait.launch
├── data/                   # Sample data point cloud (.pcd)
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## ▶️ Penggunaan

### Menjalankan Standalone (tanpa ROS)

```bash
cd build
./gait_hexapod --input ../data/sample_terrain.pcd
```

### Menjalankan dengan ROS

```bash
# Terminal 1: Start ROS Master
roscore

# Terminal 2: Launch node
roslaunch gait_hexapod_pcl hexapod_gait.launch

# Terminal 3: Publish point cloud data
rostopic pub /point_cloud sensor_msgs/PointCloud2 ...
```

### Parameter Konfigurasi

Edit file `config/params.yaml` untuk menyesuaikan parameter sistem:

```yaml
gait:
  type: "tripod"          # tripod | wave | ripple
  step_height: 0.05       # Tinggi langkah (meter)
  step_length: 0.10       # Panjang langkah (meter)
  frequency: 1.0          # Frekuensi gait (Hz)

pcl:
  voxel_size: 0.01        # Ukuran voxel untuk downsampling
  max_distance: 2.0       # Jarak maksimum deteksi (meter)
  min_cluster_size: 50    # Ukuran minimum cluster

foothold:
  search_radius: 0.05     # Radius pencarian titik pijak
  stability_margin: 0.02  # Margin stabilitas minimum
```

---

## 🧠 Algoritma

### Pipeline Pemrosesan

```
Sensor Input (LiDAR/RGB-D)
        ↓
Point Cloud Preprocessing
  - Voxel Grid Downsampling
  - Statistical Outlier Removal
        ↓
Terrain Segmentation
  - RANSAC Plane Detection
  - Euclidean Cluster Extraction
        ↓
Foothold Planning
  - Surface Normal Estimation
  - Stability Score Calculation
        ↓
Gait Generation
  - Trajectory Planning
  - Inverse Kinematics
        ↓
Motor Command Output
```

### Tipe Gait yang Didukung

| Gait | Kaki Bergerak | Kecepatan | Stabilitas |
|---|---|---|---|
| **Wave** | 1 kaki (berurutan) | Lambat | Sangat Tinggi |
| **Ripple** | 2 kaki (bergantian) | Sedang | Tinggi |
| **Tripod** | 3 kaki (alternating) | Cepat | Sedang |

---

## 📊 Hasil dan Evaluasi

Sistem ini telah diuji pada berbagai kondisi terrain:

- ✅ Permukaan datar
- ✅ Terrain miring (kemiringan hingga 15°)
- ✅ Permukaan berbatu/tidak rata
- ✅ Rintangan statis
- ⚠️ Rintangan dinamis (dalam pengembangan)

---

## 🤝 Kontribusi

Kontribusi sangat disambut! Silakan ikuti langkah berikut:

1. **Fork** repositori ini
2. Buat **branch fitur** baru: `git checkout -b feature/nama-fitur`
3. **Commit** perubahan: `git commit -m 'feat: tambahkan fitur X'`
4. **Push** ke branch: `git push origin feature/nama-fitur`
5. Buka **Pull Request**

Pastikan kode Anda mengikuti gaya penulisan yang konsisten dan disertai dokumentasi yang memadai.

---

## 📄 Lisensi

Proyek ini dilisensikan di bawah **MIT License** — lihat file [LICENSE](LICENSE) untuk detail lebih lanjut.

---

## 👤 Penulis

**Athallah Rafi**

- GitHub: [@athallahrafi](https://github.com/athallahrafi)

---

## 📚 Referensi

- Rusu, R. B., & Cousins, S. (2011). *3D is here: Point Cloud Library (PCL)*. IEEE ICRA 2011.
- PCL Documentation: https://pointclouds.org/documentation/
- Hexapod Gait Planning: Relevant robotics locomotion literature

---

<div align="center">
  <sub>Dibuat dengan ❤️ untuk riset robotika dan kecerdasan buatan</sub>
</div>
