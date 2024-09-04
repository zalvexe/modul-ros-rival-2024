# MODUL ROS 1 - Noetic (PART 2 WOWOWWOO)  

Di part 2 ini akan bahas tentang custom messages :)  

## Custom Messages  
Custom messages akan diperlukan ketika kita pingin bikin publisher sendiri dengan message yang kita custom sendiri.   
Contoh jika kita mau nerima data controller robot (velx,vely,angvel), nah kita bisa bikin message sendiri dengan isi variable tersebut :]   

__Untuk lebih memudahkan struktur file, kita bisa membuat messages sebagai 'package' sendiri yang berisi hanya folder /msg.__   

di dalam folder /msg bisa kita isi dengan file ```Controller.msg``` dengan isi

```msg
float32 velx
float32 vely
float32 angvel
```
selanjutnya untuk CMakelists, beberapa konfigurasi yang digunakan yaitu  
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(package_messages)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  Controller.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
```
selain CMakeLists, kita perlu mengatur beberapa konfigurasi pada xml sebagai berikut  
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>message_runtime</build_depend>
  <build_depend>roscpp</build_depend>
  
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>
```

## How to use it? :0   
Pada package utama kita, buat folder untuk setiap node (biar lebih rapi aja sih) dan isi dengan .cpp file untuk node nya   

Kalo udah, kita bisa lanjut bikin node publisher dan subscriber.     

### PUBLISHER
Untuk publisher yang akan menggunakan custom message yang telah dibuat, dapat diterapkan sebagai berikut   
```cpp
#include "package_messages/Controller.h"
```

Lalu kita deklarasikan Publisher
```cpp
ros::Publisher pub_pos;
```

Selanjutnya kita deklarasikan messages yang dipakai  
```cpp
package_messages::Controller Controller_msg;
```

Pada main() kita buat publisher  
```cpp
pub_controller = nh.advertise<package_messages::Controller>("Controller",10); //nh = nodehandle
```

### SUBSCRIBER  
Selanjutnya untuk node subscriber, kita bisa memanggil publisher dengan Callback()  
kita include dulu custom message
```cpp
#include "package_messages/Controller.h"
```

Selanjutnya kita deklarasi subscriber  
```cpp
ros::Subscriber sub_controller;
```

Selanjutnya kita panggil function callback  
```cpp
void controllerCallback(const package_messages::Controller::ConstPtr &Controller_msg);
```
controllerCallback bisa diisi sebagai destinasi dari message yang diterima. Untuk contoh ini, kita menyimpan hasil subscribe ke array bernama 'arr'
```cpp
void controllerCallback(const package_messages::Controller::ConstPtr &Controller_msg)
{
  memcpy(arr + 3, &Controller_msg->velx, 4); 
  memcpy(arr + 7, &Controller_msg->vely, 4);
  memcpy(arr + 11, &Controller_msg->angvel, 4);
}
```
> 4 merupakan byte size dari data float

Kemudian pada main() kita bisa tuliskan dengan   
```cpp
sub_controller = nh.subscribe("Controller", 10, controllerCallback); 
```

### CMakeLists
Untuk CMakeLists, kita hanya perlu menambahkan package custom message yang telah kita buat 
```cmake
find_package(catkin REQUIRED COMPONENTS
      cmake_modules
      roscpp
      std_msgs
      package_messages
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs package_messages
)
```

### XML
Terakhir, kita perlu modifikasi file package.xml  
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>package_messages</build_depend>
  <build_depend>message_generation</build_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>rody1_messages</build_export_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_generation</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>package_messages</exec_depend>
```
<h2 align="center">END OF PART 2.... or is it?</h2>
<p align="center">
  <img src="https://github.com/user-attachments/assets/a969e166-e017-468d-9a19-f971121819a8">
</p>
<h2 align="center">Belum ternyata.. ada dikit lagi heheheh :')</h2>

## SRV
srv (service) adalah mekanisme komunikasi yang memungkinkan node untuk melakukan panggilan prosedur jarak jauh (RPC). Komponen srv terdiri dari :   
1. Request: Bagian dari pesan yang dikirim oleh klien ke server.    
2. Response: Bagian dari pesan yang dikirim oleh server kembali ke klien.

Pemakaian srv :  
Service didefinisikan dalam file .srv. Misalnya, CalculateDistance.srv   
```CalculateDistance.srv```   
```srv
float64 speed
float64 time
---
float64 distance
```

Selanjutnya kita buat file Server dan dan Node   
#### Server Node
```cpp
#include "ros/ros.h"
#include "nama_package/CalculateDistance.h"

bool calculateDistance(nama_package::CalculateDistance::Request &req,
                       nama_package::CalculateDistance::Response &res)
{
  res.distance = req.speed * req.time;
  ROS_INFO("request: speed=%f, time=%f", req.speed, req.time);
  ROS_INFO("sending back response: distance=%f", res.distance);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_distance_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_distance", calculateDistance);
  ROS_INFO("Ready to calculate distance.");
  ros::spin();

  return 0;
}
```

#### Client Node
```cpp
#include "ros/ros.h"
#include "nama_package/CalculateDistance.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_distance_client");
  if (argc != 3)
  {
    ROS_INFO("usage: calculate_distance_client SPEED TIME");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nama_package::CalculateDistance>("calculate_distance");
  your_package_name::CalculateDistance srv;
  srv.request.speed = atof(argv[1]);
  srv.request.time = atof(argv[2]);

  if (client.call(srv))
  {
    ROS_INFO("Distance: %f", srv.response.distance);
  }
  else
  {
    ROS_ERROR("Failed to call service calculate_distance");
    return 1;
  }

  return 0;
}
```

#### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(your_package_name)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

#nambahin srv
add_service_files(
  FILES
  CalculateDistance.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calculate_distance_server src/calculate_distance_server.cpp)
add_dependencies(calculate_distance_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calculate_distance_server ${catkin_LIBRARIES})

add_executable(calculate_distance_client src/calculate_distance_client.cpp)
add_dependencies(calculate_distance_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(calculate_distance_client ${catkin_LIBRARIES})
```
<h2 align="center">END OF PART 2 (for real this time:)</h2>










