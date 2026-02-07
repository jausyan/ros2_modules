# 6. SURPRIZEEEE !!!!!!

## TASK 1

Setelah mempelajari modul-modul sebelumnya (workspace, publisher, subscriber, parameter, pengolahan data topic), sekarang saatnya menguji pemahamanmu dengan tugas project intermediate!

---

## Deskripsi Tugas

Buatlah sistem monitoring robot sederhana dengan beberapa node yang saling berkomunikasi:

### 1. Node Publisher: Sensor Current Location
- Publish data `current_location` (Float64) ke topic `/gps`
- Nilai awal bebas, bertambah setiap timer (misal +1.0 per detik)
- Tidak perlu config.yaml, langsung di script

### 2. Node Publisher: Sensor Battery
- Publish data `battery_level` (Float64) ke topic `/battery`
- Nilai awal 100, berkurang 2 setiap timer (misal 1 detik)
- Jika baterai < 20, log status "WARNING: Battery Low!"

### 3. Node Subscriber: Analisis Robot
- Subscribe ke `/gps` dan `/battery`
- Setiap menerima data, lakukan:
  - Hitung estimasi waktu bertahan: `battery_level / (current_location * 0.1)`
  - Jika estimasi < 10, log status "CRITICAL: Robot needs charging!"
  - Cetak info lengkap: current_location, battery_level, estimasi waktu

---

## TASK 2

### Intermediate Challenge: Multi-Sensor & Emergency Handling

Buat sistem robot yang terdiri dari beberapa node berikut:

#### 1. Node Publisher: Sensor Temperature
- Publish data `temperature` (Float64) ke topic `/temperature`
- Nilai random antara 25.0 - 80.0 (gunakan random generator)
- Jika temperature > 70, log status "WARNING: Overheat!"

#### 2. Node Publisher: Sensor Speed
- Publish data `speed` (Float64) ke topic `/speed`
- Nilai random antara 0.5 - 5.0

#### 3. Node Subscriber: Emergency Analyzer
- Subscribe ke `/temperature` dan `/speed`
- Jika temperature > 70 ATAU speed > 4.5, log status "CRITICAL: Emergency Stop Activated!"
- Jika temperature < 30 DAN speed < 1.0, log status "INFO: Robot in safe idle mode"
- Cetak info lengkap setiap kali menerima data

#### 4. Node Service: Emergency Stop
- Implementasikan service `/emergency_stop` (std_srvs/Empty)
- Ketika analyzer mendeteksi kondisi emergency, panggil service ini (simulasi stop)
- Log status "Robot stopped by emergency!"

## Submission

Link pengumpulan akan diinfokan mendatang di grup WA.

Lanjut ke [closing](07_closing.md) 
