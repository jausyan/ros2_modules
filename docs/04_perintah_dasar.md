# 4. Perintah-Perintah Dasar ROS 2

ROS 2 menyediakan berbagai command-line tools yang powerful untuk introspeksi, debugging, dan manajemen sistem.

## Basic Ros 2 Command
### Publish Data
```bash
ros2 run demo_nodes_cpp talker
```
maka hasilnya akan seperti ini
```bash
[INFO] [1770230462.781911987] [talker]: Publishing: 'Hello World: 1'
[INFO] [1770230463.781890298] [talker]: Publishing: 'Hello World: 2'
[INFO] [1770230464.781899374] [talker]: Publishing: 'Hello World: 3'
```
selamat kamu sudah mempublish data 'Hello World' <br>
### Subscribe Data
```bash
ros2 run demo_nodes_cpp listener
```
maka akan muncul seperti ini
```bash
[INFO] [1770230573.476843290] [listener]: I heard: [Hello World: 1]
[INFO] [1770230574.476717313] [listener]: I heard: [Hello World: 2]
[INFO] [1770230575.476822146] [listener]: I heard: [Hello World: 3]
```
data berupa kalimat 'Hello World' sudah diterima oleh listener
### Melihat Data
untuk meilhat traffic data yang sedan berjalan kita bisa melihat topic apa saja yang sedang berjalan dengan
```bash
ros2 topic list
```
perintah ini akan menunjukan topic apa saja yang sedang berjalan sekarang
```bash
/chatter # nama topic
```
untuk melihat data yang sedang dikirim
```bash
ros2 topic echo /chatter
```
ini akan menampilkan informasi data yang dikirim seperti ini
```bash
data: 'Hello World: 1'
---
data: 'Hello World: 2'
---
data: 'Hello World: 3'
```
## 1. ros2 node - Manajemen Nodes

### List Active Nodes

```bash
# Melihat semua node yang sedang berjalan
ros2 node list

# Output contoh:
# /talker
# /listener
```

### Node Info

```bash
# Melihat informasi detail tentang node
ros2 node info /talker

# Output akan menampilkan:
# - Subscribers
# - Publishers  
# - Services
# - Actions
# - Parameters
```

### Example Output:

```
/talker
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    ...
  Service Clients:

  Action Servers:

  Action Clients:
```

## 2. ros2 topic - Bekerja dengan Topics

### List Topics

```bash
# Melihat semua topic aktif
ros2 topic list

# Dengan type information
ros2 topic list -t

# Output contoh:
# /chatter [std_msgs/msg/String]
# /parameter_events [rcl_interfaces/msg/ParameterEvent]
# /rosout [rcl_interfaces/msg/Log]
```

### Topic Info

```bash
# Info tentang topic tertentu
ros2 topic info /chatter

# Output:
# Type: std_msgs/msg/String
# Publisher count: 1
# Subscription count: 1
```

### Echo Topic (Monitor Messages)

```bash
# Melihat messages yang dipublish di topic
ros2 topic echo /chatter

# Dengan limit jumlah messages
ros2 topic echo /chatter --once  # 1 message
ros2 topic echo /chatter -n 5    # 5 messages

# Dengan field tertentu
ros2 topic echo /chatter --field data
```

### Publish to Topic

```bash
# Publish message sekali
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS 2'"

# Publish secara kontinyu dengan rate tertentu (Hz)
ros2 topic pub -r 10 /chatter std_msgs/msg/String "data: 'Hello'"

# Publish sekali lalu exit
ros2 topic pub --once /chatter std_msgs/msg/String "data: 'Hello'"
```

### Topic Frequency (Hz)

```bash
# Cek frekuensi publishing
ros2 topic hz /chatter

# Output:
# average rate: 10.002
#   min: 0.099s max: 0.101s std dev: 0.00050s window: 100
```

### Topic Bandwidth (BW)

```bash
# Cek bandwidth usage
ros2 topic bw /camera/image_raw

# Berguna untuk sensor data yang besar
```

### Topic Delay

```bash
# Cek delay antara publish dan receive
ros2 topic delay /my_topic
```

## 3. ros2 interface - Message/Service/Action Types

### List Interfaces

```bash
# List semua message types
ros2 interface list

# Filter by package
ros2 interface list | grep std_msgs
ros2 interface list | grep geometry_msgs
```

### Show Interface Definition

```bash
# Melihat struktur message
ros2 interface show std_msgs/msg/String
# Output:
# string data

ros2 interface show geometry_msgs/msg/Twist
# Output:
# Vector3 linear
#   float64 x
#   float64 y
#   float64 z
# Vector3 angular
#   float64 x
#   float64 y
#   float64 z
```

### Show Service Interface

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
# Output:
# int64 a
# int64 b
# ---
# int64 sum
```

### Show Package Interfaces

```bash
# Semua interfaces dalam package
ros2 interface package std_msgs
ros2 interface package geometry_msgs
```

## 4. ros2 service - Bekerja dengan Services

### List Services

```bash
# Melihat semua service aktif
ros2 service list

# Dengan type information
ros2 service list -t
```

### Service Type

```bash
# Melihat type dari service
ros2 service type /add_two_ints

# Output:
# example_interfaces/srv/AddTwoInts
```

### Find Services by Type

```bash
# Cari service dengan type tertentu
ros2 service find example_interfaces/srv/AddTwoInts
```

### Call Service

```bash
# Memanggil service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"

# Output:
# waiting for service to become available...
# requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=10)
# response:
# example_interfaces.srv.AddTwoInts_Response(sum=15)
```

### Service Examples:

```bash
# Clear service (untuk clearing cost maps dll)
ros2 service call /clear std_srvs/srv/Empty

# Set parameter via service
ros2 service call /my_node/set_parameters rcl_interfaces/srv/SetParameters ...
```

## 5. ros2 param - Bekerja dengan Parameters

### List Parameters

```bash
# List semua parameters dari semua nodes
ros2 param list

# List parameters dari node tertentu
ros2 param list /my_node
```

### Get Parameter Value

```bash
# Get value dari parameter
ros2 param get /my_node my_param

# Output:
# String value is: default_value
```

### Set Parameter Value

```bash
# Set parameter value
ros2 param set /my_node my_param "new_value"
ros2 param set /my_node speed 2.5
ros2 param set /my_node enable_sensor true

# Output:
# Set parameter successful
```

### Dump Parameters to File

```bash
# Save parameters ke YAML file
ros2 param dump /my_node

# Atau specify filename
ros2 param dump /my_node --output-dir ./config
```

### Load Parameters from File

```bash
# Load parameters dari YAML file
ros2 param load /my_node ./params.yaml
```

### Example Parameter File (params.yaml):

```yaml
my_node:
  ros__parameters:
    my_param: "hello"
    speed: 1.5
    enable_sensor: true
```

## 6. ros2 run - Menjalankan Nodes

### Basic Run

```bash
# Menjalankan node
ros2 run <package_name> <executable_name>

# Contoh:
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### Run dengan Arguments

```bash
# Set parameters
ros2 run my_package my_node --ros-args -p my_param:=value

# Remap topic
ros2 run my_package my_node --ros-args -r chatter:=my_topic

# Set namespace
ros2 run my_package my_node --ros-args -r __ns:=/robot1

# Set node name
ros2 run my_package my_node --ros-args -r __node:=my_custom_name

# Load parameters from file
ros2 run my_package my_node --ros-args --params-file ./params.yaml
```

### Combined Arguments:

```bash
ros2 run my_package my_node --ros-args \
  -r __ns:=/robot1 \
  -r __node:=custom_name \
  -r chatter:=new_topic \
  -p speed:=2.0 \
  --params-file ./config.yaml
```

## 7. ros2 launch - Launch Multiple Nodes

```bash
# Menjalankan launch file
ros2 launch <package_name> <launch_file>

# Contoh:
ros2 launch my_package my_launch.py

# Dengan arguments
ros2 launch my_package my_launch.py param1:=value1
```

## 8. ros2 pkg - Package Management

### List Packages

```bash
# List semua installed packages
ros2 pkg list

# Search package
ros2 pkg list | grep my_package
```

### Package Info

```bash
# Show package path
ros2 pkg prefix my_package

# List executables dalam package
ros2 pkg executables my_package
```

### Create Package

```bash
# Create Python package
ros2 pkg create --build-type ament_python my_package

# Create C++ package
ros2 pkg create --build-type ament_cmake my_package

# Dengan dependencies
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs geometry_msgs
```

## 9. ros2 bag - Record & Playback

### Record Data

```bash
# Record semua topics
ros2 bag record -a

# Record topic tertentu
ros2 bag record /chatter /camera/image

# Record dengan nama file
ros2 bag record -o my_bag /chatter

# Record dengan duration limit
ros2 bag record -d 60 /chatter  # 60 detik
```

### Playback Data

```bash
# Playback recorded bag
ros2 bag play my_bag

# Playback dengan loop
ros2 bag play -l my_bag

# Playback dengan rate berbeda
ros2 bag play -r 2.0 my_bag  # 2x speed
```

### Bag Info

```bash
# Informasi tentang bag file
ros2 bag info my_bag

# Output:
# Files:             my_bag.db3
# Bag size:          1.2 MB
# Storage id:        sqlite3
# Duration:          10.5s
# Start:             Feb  5 2026 10:30:00.00
# End:               Feb  5 2026 10:30:10.50
# Messages:          105
# Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 105
```

## 10. ros2 doctor - System Check

```bash
# Comprehensive system check
ros2 doctor

# Detailed report
ros2 doctor --report

# Check specific issues
ros2 doctor --include-warnings
```

## 11. rqt - GUI Tools

### Launch rqt

```bash
# Main rqt application
rqt

# Specific rqt plugins:

# Graph visualization
rqt_graph

# Console (log viewer)
rqt_console

# Plot data
rqt_plot /my_topic/data

# Image viewer
rqt_image_view

# Service caller
rqt_service_caller

# Topic monitor
rqt_topic
```

## 12. Visualization - RViz2

```bash
# Launch RViz2
rviz2

# Dengan config file
rviz2 -d my_config.rviz
```

## Kombinasi Perintah Berguna

### 1. Complete System Info

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list -t

# List all services
ros2 service list -t

# Check system health
ros2 doctor
```

### 2. Debugging Communication

```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Monitor topic
ros2 topic echo /chatter

# Terminal 3: Check frequency
ros2 topic hz /chatter

# Terminal 4: Check node info
ros2 node info /talker
```

### 3. Testing Service

```bash
# Terminal 1: Run service server
ros2 run my_package service_server

# Terminal 2: Call service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"
```

### 4. Parameter Management

```bash
# Get all parameters
ros2 param list /my_node

# Dump to file
ros2 param dump /my_node > params.yaml

# Modify file, then load
ros2 param load /my_node params.yaml
```

## Command Cheat Sheet

| Task | Command |
|------|---------|
| List nodes | `ros2 node list` |
| List topics | `ros2 topic list` |
| Echo topic | `ros2 topic echo /topic` |
| Publish to topic | `ros2 topic pub /topic msg_type "data"` |
| Call service | `ros2 service call /srv srv_type "request"` |
| Set parameter | `ros2 param set /node param value` |
| Run node | `ros2 run pkg node` |
| Show message | `ros2 interface show msg_type` |
| Record bag | `ros2 bag record -a` |
| Play bag | `ros2 bag play bag_name` |
| Graph view | `rqt_graph` |
| System check | `ros2 doctor` |

## Tab Completion

ROS 2 mendukung tab completion:

```bash
# Enable tab completion (jika belum)
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Gunakan TAB untuk auto-complete
ros2 <TAB>        # List verbs
ros2 topic <TAB>  # List sub-commands
ros2 topic echo <TAB>  # List available topics
```

## Rangkuman

 **Perintah Utama:**
- `ros2 node` - Manajemen nodes
- `ros2 topic` - Bekerja dengan topics
- `ros2 service` - Call services
- `ros2 param` - Parameter management
- `ros2 run` - Jalankan nodes
- `ros2 launch` - Launch multiple nodes
- `ros2 bag` - Record & playback
- `rqt` - GUI tools

 **Best Practices:**
- Gunakan `ros2 doctor` untuk health check
- Gunakan `rqt_graph` untuk visualisasi
- Record important data dengan `ros2 bag`
- Monitor dengan `ros2 topic echo` dan `hz`

## Next Steps

Lanjut ke [Membuat Workspace ROS 2](05_membuat_workspace.md) untuk mulai coding!

---

**💡 Tips**: 
- Bookmark cheat sheet ini
- Eksperimen dengan demo nodes
- Gunakan `--help` pada setiap command untuk detail
