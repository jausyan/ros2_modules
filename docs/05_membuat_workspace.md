# 5. Membuat Workspace ROS 2 (C++ Sederhana)

Pada modul ini kita akan membuat contoh **workspace ROS 2** yang sangat sederhana dengan **satu package C++**. Di dalam package tersebut kita akan membuat node yang:

- Menyimpan **current location** (posisi sekarang)
- Menyimpan **goal location** (tujuan)
- Menghitung **distance = goal location - current location**
- Mempublish informasi tersebut ke sebuah topic

Contoh ini dibuat sesederhana mungkin agar mudah dipahami pemula.

---

## 1. Persiapan Workspace

Pastikan ROS 2 Jazzy sudah terinstall dan environment sudah di-*source*.

```bash
source /opt/ros/jazzy/setup.bash
```

Sekarang buat workspace baru khusus untuk contoh ini.

```bash
# Buat folder workspace
mkdir -p ~/ros2_cpp_ws/src
cd ~/ros2_cpp_ws
```

Struktur awal:

```bash
ros2_cpp_ws/
└── src/
```

---

## 2. Membuat Package C++ Sederhana

Kita akan membuat package C++ bernama `simple_location_cpp`.

```bash
cd ~/ros2_cpp_ws/src

ros2 pkg create \
	--build-type ament_cmake \
	simple_location_cpp \
	--dependencies rclcpp std_msgs
```

Perintah di atas akan membuat struktur seperti ini:

```bash
simple_location_cpp/
├── CMakeLists.txt
├── include/
│   └── simple_location_cpp/
├── package.xml
└── src/
```

Kita akan menaruh kode C++ di folder `src/`.

buat publisher sederhana [klik disini](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

---

## 3. Konsep Program yang Akan Kita Buat

Sebelum menulis kode, pahami dulu konsep sederhananya.

- Kita anggap posisi robot hanya **1 dimensi** (misalnya sumbu X saja) supaya mudah.
- Ada tiga nilai penting:
	- `current_location` : posisi sekarang (float, misalnya meter)
	- `goal_location` : posisi tujuan (float)
	- `distance` : selisih `goal_location - current_location`
- Node akan:
	- Menyimpan nilai `current_location` dan `goal_location`
	- Menghitung `distance`
	- Mempublish string sederhana berisi tiga nilai tersebut ke topic `/location_info`

Untuk pemula, kita pakai **satu topic** dan **satu jenis pesan**: `std_msgs::msg::String`.

---

## 4. Menulis Node C++: Location Publisher

Buka file `src/location_publisher.cpp` (buat file baru).

```bash
cd ~/ros2_cpp_ws/src/simple_location_cpp
touch src/location_publisher.cpp
```

Isi file `src/location_publisher.cpp` dengan kode berikut:

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LocationPublisher : public rclcpp::Node
{
public:
	LocationPublisher()
	: Node("location_publisher"),
		current_location_(0.0),
		goal_location_(10.0)  // misalnya goal di posisi 10.0 meter
	{
		// Publisher ke topic /location_info dengan tipe std_msgs::msg::String
		publisher_ = this->create_publisher<std_msgs::msg::String>("/location_info", 10);

		// Timer untuk publish setiap 1 detik
		timer_ = this->create_wall_timer(
			1000ms, std::bind(&LocationPublisher::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "LocationPublisher node started");
	}

private:
	void timer_callback()
	{
		// Hitung jarak: distance = goal - current
		double distance = goal_location_ - current_location_;

		// Buat pesan string sederhana
		auto message = std_msgs::msg::String();
		message.data = "current = " + std::to_string(current_location_)
								 + ", goal = " + std::to_string(goal_location_)
								 + ", distance = " + std::to_string(distance);

		// Publish
		publisher_->publish(message);

		// Log ke terminal
		RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

		// Untuk contoh sederhana, kita misalkan robot bergerak maju 0.5 meter per detik
		current_location_ += 0.5;
	}

	// Publisher dan timer
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Nilai posisi
	double current_location_;
	double goal_location_;
};

int main(int argc, char * argv[])
{
	// Inisialisasi ROS 2
	rclcpp::init(argc, argv);

	// Buat node dan jalankan
	rclcpp::spin(std::make_shared<LocationPublisher>());

	// Shutdown
	rclcpp::shutdown();
	return 0;
}
```

Penjelasan singkat:

- Node bernama `location_publisher`
- Setiap 1 detik:
	- Hitung `distance = goal_location - current_location`
	- Print dan publish string ke topic `/location_info`
	- Update `current_location_` dengan menambah 0.5 (seolah robot maju)

Ini sudah cukup untuk memahami konsep **node**, **publisher**, dan **topic** di ROS 2 menggunakan C++.

---

## 5. Mendaftarkan Executable di CMakeLists.txt

Sekarang kita perlu memberi tahu sistem build bahwa kita punya executable baru.

Edit file `simple_location_cpp/CMakeLists.txt` dan tambahkan bagian berikut (sesuaikan jika sudah ada template bawaan):

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_location_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(location_publisher src/location_publisher.cpp)
ament_target_dependencies(location_publisher rclcpp std_msgs)

install(TARGETS
	location_publisher
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

Poin penting:
- `add_executable(location_publisher ...)` → nama executable
- `ament_target_dependencies(...)` → daftar dependency
- `install(TARGETS ...)` → agar bisa dijalankan dengan `ros2 run`

---

## 6. Build Workspace

Sekarang kita kembali ke root workspace dan melakukan build.

```bash
cd ~/ros2_cpp_ws

colcon build
```

Jika build sukses, akan muncul folder `build/`, `install/`, dan `log/`.

Setelah build selesai, *source* workspace:

```bash
source install/setup.bash
```

Tips: supaya tidak perlu `source` berkali-kali, bisa tambahkan ke `~/.bashrc`:

```bash
echo "source ~/ros2_cpp_ws/install/setup.bash" >> ~/.bashrc
```

---

## 7. Menjalankan Node dan Melihat Output

### Menjalankan Node

Pastikan sudah `source` ROS 2 dan workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_cpp_ws/install/setup.bash

ros2 run simple_location_cpp location_publisher
```

Jika berhasil, di terminal akan muncul log seperti:

```text
[INFO] [xxxx.xx] [location_publisher]: current = 0.000000, goal = 10.000000, distance = 10.000000
[INFO] [xxxx.xx] [location_publisher]: current = 0.500000, goal = 10.000000, distance = 9.500000
[INFO] [xxxx.xx] [location_publisher]: current = 1.000000, goal = 10.000000, distance = 9.000000
...
```

### Melihat Data dengan `ros2 topic echo`

Buka terminal kedua dan *source* environment lagi, lalu jalankan:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_cpp_ws/install/setup.bash

ros2 topic list
```

Kamu akan melihat topic `/location_info`. Untuk melihat isi pesannya:

```bash
ros2 topic echo /location_info
```

Outputnya kurang lebih sama dengan log yang dicetak node.

---

## 8. Ringkasan Konsep yang Dipakai

Di contoh ini, kamu sudah mempraktikkan beberapa konsep dasar ROS 2 dengan C++:

- **Workspace**: folder `~/ros2_cpp_ws` dengan struktur `src/`, `build/`, `install/`, `log/`
- **Package**: `simple_location_cpp` sebagai wadah kode C++
- **Node**: `LocationPublisher` yang berjalan sebagai proses ROS 2
- **Publisher**: mengirim data ke topic `/location_info`
- **Topic**: jalur komunikasi tempat data dikirim dan bisa disubscribe node lain

Walaupun sederhana, pola ini sama dengan pola yang dipakai di robot nyata:

- Sensor node → publish data (posisi, kecepatan, dsb.)
- Processing node → hitung jarak ke goal, generate command
- Control node → kirim perintah ke motor/flight controller

---

## 9. Latihan untuk Kamu

Untuk memperkuat pemahaman, coba modifikasi contoh ini:

1. **Tambahkan kondisi berhenti**
	 - Jika `distance` sudah lebih kecil dari 0.1, berhenti menambah `current_location_`.

Dengan latihan-latihan kecil ini, kamu akan semakin nyaman dengan konsep node, publisher, topic, dan cara build project C++ di ROS 2.

---

## 10. Menggunakan `config.yaml` untuk Mengatur Nilai (Tanpa Rebuild)

Pada contoh sebelumnya, nilai-nilai berikut masih **di-hardcode** di dalam kode C++:

- `current_location_` (awal = `0.0`)
- `goal_location_` (awal = `10.0`)
- `speed` (laju gerak = `0.5`)

Sekarang kita akan memindahkan nilai-nilai ini ke file konfigurasi `config/config.yaml` dan membaca nilainya sebagai **parameter ROS 2**. Tujuannya:

- Jika ingin mengubah nilai, **tidak perlu build ulang** workspace.
- Cukup ubah file `.yaml`, lalu jalankan node dengan membawa file config tersebut.

### 10.1. Menyiapkan Folder dan File `config.yaml`

Di dalam package `simple_location_cpp`, buat folder `config` dan file `config.yaml`:

```bash
cd ~/ros2_cpp_ws/src/simple_location_cpp
mkdir -p config
touch config/config.yaml
```

Isi file `config/config.yaml` dengan isi seperti berikut:

```yaml
/**:
  ros__parameters:
    current_location: 100.0
    goal_location: 500.0
    speed: 10.0
```

Keterangan:

- `simple_location_cpp:` → namespace konfigurasi untuk package ini.
- `ros__parameters:` → bagian standar untuk mendefinisikan parameter ROS 2.
- `current_location`, `goal_location`, `speed` → nilai yang ingin kita atur tanpa rebuild.

Kamu bebas mengubah angka-angka ini nanti, misalnya `goal_location: 5.0` atau `speed: 1.0`.

### 10.2. Update Kode C++ agar Membaca Parameter

Sekarang kita ubah `location_publisher.cpp` supaya **membaca parameter** dari ROS 2, bukan nilai tetap.

Buka lagi file `src/location_publisher.cpp` dan ubah bagian konstruktor seperti ini:

```cpp
class LocationPublisher : public rclcpp::Node
{
public:
	LocationPublisher()
	: Node("location_publisher")
	{
		// Deklarasi parameter dengan nilai default
		this->declare_parameter<double>("current_location", 0.0);
		this->declare_parameter<double>("goal_location", 10.0);
		this->declare_parameter<double>("speed", 0.5);

		// Membaca nilai parameter dari server parameter
		current_location_ = this->get_parameter("current_location").as_double();
		goal_location_ = this->get_parameter("goal_location").as_double();
		speed_ = this->get_parameter("speed").as_double();

		// Publisher ke topic /location_info dengan tipe std_msgs::msg::String
		publisher_ = this->create_publisher<std_msgs::msg::String>("/location_info", 10);

		// Timer untuk publish setiap 1 detik
		timer_ = this->create_wall_timer(
			1000ms, std::bind(&LocationPublisher::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "LocationPublisher node started");
		RCLCPP_INFO(this->get_logger(), "Start current=%.2f, goal=%.2f, speed=%.2f",
			current_location_, goal_location_, speed_);
	}

private:
	void timer_callback()
	{
		// Hitung jarak: distance = goal - current
		double distance = goal_location_ - current_location_;

		// Buat pesan string sederhana
		auto message = std_msgs::msg::String();
		message.data = "current = " + std::to_string(current_location_)
					 + ", goal = " + std::to_string(goal_location_)
					 + ", distance = " + std::to_string(distance);

		// Publish
		publisher_->publish(message);

		// Log ke terminal
		RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

		// Update posisi berdasarkan speed
		current_location_ += speed_;
	}

	// Publisher dan timer
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Nilai posisi dan kecepatan
	double current_location_;
	double goal_location_;
	double speed_;
};
```

Perbedaan utama dengan versi sebelumnya:

- Ada parameter baru `speed_`.
- `current_location_`, `goal_location_`, dan `speed_` **dibaca dari parameter**.
- Kode `current_location_ += 0.5;` diganti menjadi `current_location_ += speed_;`.

> Catatan: Setelah mengubah kode C++, **sekali ini saja** perlu `colcon build` lagi karena kita mengubah source code.

### 10.3. Menjalankan Node dengan `config.yaml`

Ada dua cara umum untuk menggunakan parameter dari file `.yaml`:

#### Cara A: Menggunakan `ros2 run` + argumen `--ros-args`

Pertama, pastikan sudah build dan source workspace:

```bash
cd ~/ros2_cpp_ws
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Lalu jalankan node dengan membawa file konfigurasi:

```bash
ros2 run simple_location_cpp location_publisher \
	--ros-args --params-file ~/ros2_cpp_ws/src/simple_location_cpp/config/config.yaml
```

Sekarang nilai `current_location`, `goal_location`, dan `speed` akan dibaca dari `config.yaml`.

Jika kamu mengubah isi `config.yaml` (misalnya `goal_location: 20.0` atau `speed: 1.0`), **tidak perlu build ulang**. Cukup:

```bash
edit ~/ros2_cpp_ws/src/simple_location_cpp/config/config.yaml

ros2 run simple_location_cpp location_publisher \
	--ros-args --params-file ~/ros2_cpp_ws/src/simple_location_cpp/config/config.yaml
```

#### Cara B (Lanjutan): Menggunakan Launch File

Di modul lain, kita bisa membuat **launch file** yang otomatis memuat `config.yaml`. Untuk sekarang, cara A sudah cukup untuk pemula.

---

Dengan menggunakan `config.yaml` dan parameter ROS 2, kamu sudah belajar cara:

- Memisahkan **konfigurasi** dari **kode**.
- Mengubah perilaku node (start position, goal, speed) tanpa build ulang.
- Menjalankan node dengan file parameter menggunakan `--ros-args --params-file`.

Ini adalah pola yang sangat umum dipakai di project robot nyata, misalnya untuk mengatur PID, batas kecepatan, atau target posisi tanpa harus mengubah dan membuild ulang kode C++.

---

## 11. Mengolah Data dari Topic: Subscriber Kalikan 2

Setelah kamu bisa **mempublish** data dan mengatur nilai lewat **parameter/config**, langkah berikutnya adalah belajar **mengolah data dari topic**.

Di robot nyata biasanya ada pola seperti ini:

- Node **sensor** → publish data mentah (misalnya posisi, kecepatan, jarak, dsb.).
- Node **processing** → subscribe ke data tersebut, melakukan perhitungan, lalu mengirim hasilnya.

Pada contoh ini kita akan membuat:

1. **Publisher** yang mengirim data `current_location` (Float64) ke topic `/gps`
2. **Subscriber** yang menerima data dari `/gps`, mengalikan dengan 2, lalu mencetak hasilnya

Konsep yang dipelajari: **publish data → subscribe → olah data (× 2)**.

### 11.1. Membuat Package Baru

Kita akan membuat package baru bernama `gps_processor` untuk contoh ini:

```bash
cd ~/ros2_cpp_ws/src

ros2 pkg create \
	--build-type ament_cmake \
	gps_processor \
	--dependencies rclcpp std_msgs
```

### 11.2. Publisher: Mengirim Current Location

Buat file `src/gps_publisher.cpp`:

```bash
cd ~/ros2_cpp_ws/src/gps_processor
touch src/gps_publisher.cpp
```

Isi dengan kode berikut:

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class GpsPublisher : public rclcpp::Node
{
public:
	GpsPublisher()
	: Node("gps_publisher"),
		current_location_(0.0)  // Posisi awal 0.0
	{
		// Publisher ke topic /gps dengan tipe Float64
		publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gps", 10);

		// Timer publish setiap 1 detik
		timer_ = this->create_wall_timer(
			1000ms, std::bind(&GpsPublisher::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "GPS Publisher started");
	}

private:
	void timer_callback()
	{
		auto msg = std_msgs::msg::Float64();
		msg.data = current_location_;

		publisher_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Publishing current_location: %.2f", current_location_);

		// Update posisi: tambah 1.0 setiap detik
		current_location_ += 1.0;
	}

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	double current_location_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GpsPublisher>());
	rclcpp::shutdown();
	return 0;
}
```

**Penjelasan:**

- Node `gps_publisher` publish nilai `current_location` ke topic `/gps`
- Tipe data: `std_msgs::msg::Float64` (angka float/double)
- Nilai dimulai dari 0.0 dan bertambah 1.0 setiap detik
- **Tidak pakai config.yaml**, semua nilai langsung di kode

### 11.3. Subscriber: Mengolah Data (× 2)

Buat file `src/gps_subscriber.cpp`:

```bash
cd ~/ros2_cpp_ws/src/gps_processor
touch src/gps_subscriber.cpp
```

Isi dengan kode berikut:

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class GpsSubscriber : public rclcpp::Node
{
public:
	GpsSubscriber()
	: Node("gps_subscriber")
	{
		// Subscribe ke topic /gps
		subscription_ = this->create_subscription<std_msgs::msg::Float64>(
			"/gps", 10,
			std::bind(&GpsSubscriber::topic_callback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "GPS Subscriber started, waiting for /gps data...");
	}

private:
	void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
	{
		// Ambil data current_location dari topic
		double current_location = msg->data;

		// Kalikan dengan 2
		double location_x2 = current_location * 2.0;

		// Tampilkan hasil
		RCLCPP_INFO(this->get_logger(),
			"Received: %.2f | Processed (x2): %.2f",
			current_location, location_x2);
	}

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GpsSubscriber>());
	rclcpp::shutdown();
	return 0;
}
```

**Penjelasan:**

- Node `gps_subscriber` subscribe ke topic `/gps`
- Setiap terima data, ambil nilai dari `msg->data`
- Kalikan dengan 2: `location_x2 = current_location * 2.0`
- Cetak hasil: nilai asli dan hasil perkalian

### 11.4. Update CMakeLists.txt

Edit file `gps_processor/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(gps_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Executable untuk publisher
add_executable(gps_publisher src/gps_publisher.cpp)
ament_target_dependencies(gps_publisher rclcpp std_msgs)

# Executable untuk subscriber
add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber rclcpp std_msgs)

install(TARGETS
	gps_publisher
	gps_subscriber
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### 11.5. Build dan Menjalankan

Build workspace:

```bash
cd ~/ros2_cpp_ws
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Jalankan Publisher** (terminal 1):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_cpp_ws/install/setup.bash

ros2 run gps_processor gps_publisher
```

**Jalankan Subscriber** (terminal 2):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_cpp_ws/install/setup.bash

ros2 run gps_processor gps_subscriber
```

### 11.6. Output yang Diharapkan

**Terminal 1 (publisher):**

```text
[INFO] [gps_publisher]: GPS Publisher started
[INFO] [gps_publisher]: Publishing current_location: 0.00
[INFO] [gps_publisher]: Publishing current_location: 1.00
[INFO] [gps_publisher]: Publishing current_location: 2.00
...
```

**Terminal 2 (subscriber):**

```text
[INFO] [gps_subscriber]: GPS Subscriber started, waiting for /gps data...
[INFO] [gps_subscriber]: Received: 0.00 | Processed (x2): 0.00
[INFO] [gps_subscriber]: Received: 1.00 | Processed (x2): 2.00
[INFO] [gps_subscriber]: Received: 2.00 | Processed (x2): 4.00
...
```

### 11.7. Konsep yang Sudah Dipelajari

Dengan contoh sederhana ini, kamu sudah belajar:

**Publisher** mengirim data float ke topic  
**Subscriber** menerima data dari topic  
**Mengolah data** yang diterima (kalikan 2)  
**Komunikasi antar node** melalui topic  

Di robot nyata, pola ini sangat umum:
- GPS sensor → publish koordinat → Navigation node hitung jarak
- Kamera → publish image → Vision node deteksi objek
- Lidar → publish point cloud → Mapping node buat peta

### 11.8. Latihan untuk Kamu

Coba modifikasi untuk memperdalam pemahaman:

1. **Ubah operasi**: Ganti `× 2` menjadi `÷ 2` atau `+ 10`
2. **Tambah kondisi**: Hanya proses jika `current_location > 5.0`
3. **Publish hasil**: Buat publisher di subscriber yang kirim hasil ke topic `/processed_gps`

Lanjut ke [suprizeee](06_suprizeee.md) untuk memulai setup environment Anda!
