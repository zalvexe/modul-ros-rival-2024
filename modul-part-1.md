# ROS 1 - Noetic Modul 2024

## Apa itu ROS??  
ROS (Robot Operating System) adalah sebuah kerangka kerja yang dirancang untuk membantu pengembangan perangkat lunak robot. Meski namanya "Operating System", sebenarnya ROS bukanlah sistem operasi seperti Windows atau Linux. Sebaliknya, ROS adalah sekumpulan alat, pustaka, dan konvensi yang membantu para pengembang dalam membangun perangkat lunak robot.

Dalam dunia pengembangan robot, ada banyak komponen yang harus berkomunikasi satu sama lain, seperti sensor, aktuator, dan sistem pemrosesan data. ROS menyediakan infrastruktur untuk mempermudah komunikasi ini, sehingga pengembang bisa lebih fokus pada logika atau fungsi robot itu sendiri, daripada memikirkan bagaimana komponen-komponen tersebut saling terhubung.

![image](https://github.com/user-attachments/assets/0bc3f43f-cc55-486e-9354-8350a6690132)

ROS bikin hidup lebih mudah bagi para pengembang robot dengan menyediakan:
- **Library dan Tools**: Ini kaya kotak alat yang penuh dengan kode siap pakai untuk berbagai keperluan robot, seperti pengolahan gambar, kontrol motor, dan komunikasi.
- **Framework**: ROS memberikan struktur yang konsisten untuk membuat dan menjalankan berbagai aplikasi robot.
- **Komunikasi**: Dengan ROS, komponen robot yang berbeda bisa ngobrol satu sama lain dengan mudah, misalnya, sensor yang mengirim data ke prosesor yang kemudian memerintahkan motor.

Jadi, kalau kamu pengen bikin robot yang pintar, fleksibel, dan bisa berkomunikasi dengan komponen lain, ROS adalah alat yang bisa sangat membantu! :]   
     
## Struktur ROS 

![image](https://github.com/user-attachments/assets/5a85cc9c-f80c-4224-9df7-90751d3da54d)

Komponen utama pada ROS terdiri dari :  
__1. Node:__

Node adalah unit eksekusi dasar dalam ROS. Setiap node adalah proses yang melakukan tugas tertentu, seperti membaca sensor, mengendalikan aktuator, atau melakukan komputasi.
Node berkomunikasi satu sama lain menggunakan topik, layanan, dan parameter.

__2. Master:__

ROS Master bertindak sebagai pendaftar pusat yang mengelola informasi tentang node, topik, dan layanan.
Master memungkinkan node untuk menemukan satu sama lain dan berkomunikasi. Tanpa Master, node ngga bisa berkomunikasi.

__3. Parameter Server (rosparam):__

Parameter Server adalah penyimpanan pusat untuk parameter konfigurasi yang dapat diakses oleh semua node. Parameter dapat digunakan untuk menyimpan konfigurasi yang dapat diubah tanpa perlu memodifikasi kode sumber.

__4. Topic:__

Topik adalah mekanisme komunikasi yang digunakan untuk pertukaran pesan antar node secara asinkron.
Node dapat menerbitkan (publish) pesan ke topik atau berlangganan (subscribe) ke topik untuk menerima pesan.

__5. Service:__

Layanan (service) adalah mekanisme komunikasi sinkron yang memungkinkan node untuk melakukan panggilan prosedur jarak jauh (RPC). Layanan terdiri dari permintaan (request) dan tanggapan (response).

__6. Message:__
Pesan (message) adalah struktur data yang digunakan untuk pertukaran informasi antar node melalui topik atau layanan. Pesan didefinisikan menggunakan file .msg yang menentukan tipe data dan struktur pesan.

## 1. Instalasi ROS 1 - Noetic
[link untuk penjelasan lebih lengkap](https://wiki.ros.org/noetic/Installation/Ubuntu)  
Untuk instalasi ROS1 Noetic, pastikan Operating System yang digunakan merupakan Ubuntu 20.04 oke :D   

### __Pra-instalasi :__      
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```  

Install curl dulu (kalo belum)   
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```    


### __Memulai instalasi :__   
Update sistem dulu man teman   
```
sudo apt update && sudo apt upgrade  
```   

Install full desktop :   
```
sudo apt install ros-noetic-desktop-full
```   


### __Environment setup (buat sourcing) :__    
(buat bash terminal) :
```
source /opt/ros/noetic/setup.bash
```   

(buat bash lain, e.g. zsh) : 
```
source /opt/ros/noetic/setup.bash
```   

Lanjut kita tambahin source ke bashrc atau zshrc :  
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```  

```
source ~/.bashrc
```   
> untuk zsh bisa langsung diganti setup.zsh dan .zshrc


### __Install Dependencies untuk Keperluan Building Package Python:__   
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep  

sudo rosdep init

rosdep update
```   
## 2. Tes ROS dengan Turtlesim, Topics
Kita coba jalanin package turtlesim dari ROS..  
untuk run beberapa nodes, kita perlu beberapa terminal:')   

Terminal pertama, kita jalankan roscore sebagai master    
```
roscore
```
Lanjut kita buka terminal kedua untuk turtlesim  
```
rosrun turtlesim turtlesim_node
```
Setelah jalanin command tersebut, akan muncul window turtlesim  

![image](https://github.com/user-attachments/assets/8454843a-9bdf-4e21-ae98-a909232d356e)   
Tapi, kita belum bisa mengontrol turtle tersebut.. jadi kita perlu memanggil node lain sebagai controller  
```
rosrun turtlesim turtle_teleop_key
```
Sekarang, kita bisa mengontrol turtle dengan keyboard:D   

![image](https://github.com/user-attachments/assets/200cd234-489b-4f99-976b-0739b684a488)    
Dari sini, kita bisa coba cek topic yang berjalan...    

![image](https://github.com/user-attachments/assets/9337b7d5-4f93-427c-b6bd-b1923bcd3426)

Kita bisa cek topic dan apa yang dikirim/diterima suatu node dengan command berikut   
```
rostopic list
rostopic echo [nama topic]
```

## 3. Coba Bikin Package  
Lanjut nih.. kita coba bikin simple publisher & subscriber  

Bikin workspace & package duluu  
```
mkdir -p ~/tutorial_ws/src  
cd ~/tutorial_ws/src
catkin_create_pkg simple_demo_cpp roscpp std_msgs
cd ~/tutorial_ws
catkin_make
``` 
> catkin_make itu buat build workspace yg kita bikin tadi

Lanjut kita sourcing, sourcing digunakan untuk set up environment yang digunakan di package dan executable file yang kita build nantinya  

```
source devel/setup.bash
```   
Kalo udah, kita masuk ke directory ```/src``` dan bikin node talker & listener   

```talker.cpp```
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(1); // 1 Hz
  int cnt = 0; 
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello ROS " << cnt++ << ros::Time::now();
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
```
> loop_rate(1) mengartikan loop program berulang setiap 1 detik. Nilai loop_rate ini berbeda di program, untuk program yang kompleks biasanya digunakan loop_rate yang lebih cepat dan umumnya menggunakan ros timer

```listener.cpp```
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
```
### Apa yang dilakukan program?  
Jadi, kedua program tersebut merupakan node talker dan node listener. Talker akan mempublish data berupa "Hello ROS" yang akan diterima oleh listener (kita ngambil informasi dari publisher lewat Callback function)  

Lanjut nih.. kita masuk ke file ```CMakeLists.txt```. CMakeLists ini berguna untuk mendefinisikan konfigurasi build yang digunakan di package kita   

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(simple_demo_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```
dalam project() diisi dengan nama package   
find_package() diisi dengan package-package yang kita gunakan di workspace   

> CMakeLists ini pasti akan berbeda di setiap project, masih ada beberapa function di CMakeLists yang belum terpakai di sini :"]   


### Build Program   
Untuk memastikan program kita udah bener, kita coba build dengan catkin_make  
```
cd ~/tutorial_ws/
catkin_make
```

### Run program
Untuk run program ROS, kita akan perlu beberapa terminal :")     
terminal pertama kita pake buat roscore   
```
roscore
```
lanjut.. terminal kedua kita run node talker   
```
rosrun tutorial_ws talker
```
terminal ketiga kita run node listener  
```
rosrun tutorial_ws listener
```
![image](https://github.com/user-attachments/assets/563feaf6-3e25-4e24-9882-6b9cb123beb8)

kita bisa lihat bahwa listener dan talker saling menerima dan mengirim dengan jumlah count yang sinkron :D   

<h2 align="center">--- END OF PART 1 ---</h2>
<p align="center">
     <img src="https://github.com/user-attachments/assets/9b8c5daf-c66c-4216-a5bf-322112e92bc3">
</p>
