<h1 id="ros2-ile-amr-gelistirme-el-kitabi">ROS2 ile AMR Geliştirme El Kitabı</h1>

> **📌 GARPamr_edu** - Geliştirilmiş ve Tamamlanmış Versiyon

---

<h2 id="fihrist">Fihrist</h2>

1. **[Önsöz – Amaç](#hid-1)**

2. **[Konumlama](#hid-2)**  
  2.1 [Temel Kavramlar](#hid-2-1)  
  2.2 [Transform (Frame'ler Arasındaki İlişki)](#hid-2-2)  
  2.3 [Frame Ağacı (TF Tree)](#hid-2-3)

3. **[Modelleme (URDF)](#hid-3)**  
  3.1 [Temel Kavramlar](#hid-3-1)  
  3.2 [Link Anlatısı](#hid-3-2)  
  3.3 [Joint Anlatısı](#hid-3-3)  
  3.4 [URDF Anlatısı](#hid-3-4)  
  3.5 [Modelin ROS2'ye Aktarımı](#hid-3-5)  
  3.6 [Model Örneği](#hid-3-6)  
  3.7 [Xacro Aracı](#hid-3-7)

4. **[Simülasyon (Gazebo)](#hid-4)**  
  4.1 [SDF (Simulation Definition Format)](#hid-4-1)  
  4.2 [SDF - URDF Etkileşimi](#hid-4-2)  
  4.3 [Gazebo - ROS Bağlantısı](#hid-4-3)  
  4.4 [Pluginler](#hid-4-4)  
  4.5 [Sensörler](#hid-4-5)  
  4.6 [Dünya Oluşturma](#hid-4-6)  
  4.7 [Model Oluşturma](#hid-4-7)

5. **[Haritalama (slam_toolbox)](#hid-5)**   
  5.1 ['slam_toolbox' Çalışma Mantığı](#hid-5-1)  
  5.2 [Online-Offline, Sync-Async Mantığı](#hid-5-2)  
  5.3 [Parametreler](#hid-5-3)  
  5.4 [Genel Kullanım](#hid-5-4)

6. **[Navigasyon (nav2)](#hid-6)**  
  6.1 ['nav2' Çalışma Mantığı](#hid-6-1)  
  6.2 [AMCL Detaylı Anlatım](#hid-6-2)  
  6.3 [Costmap Yapısı](#hid-6-3)  
  6.4 [Planner ve Controller](#hid-6-4)  
  6.5 [Parametreler](#hid-6-5)  
  6.6 [Genel Kullanım](#hid-6-6)

7. **[Simülasyon vs Gerçeklik](#hid-7)**  
  7.1 [Benzerlik ve Farklılıklar](#hid-7-1)  
  7.2 [Temel Sistem Tasarımı](#hid-7-2)  
  7.3 [Üst Sistem Gerçekleştirme](#hid-7-3)  
  7.4 [Alt Sistem Gerçekleştirme](#hid-7-4)  
  7.5 [micro-ROS Kullanımı](#hid-7-5)  
  7.6 [Dikkat Edilmesi Gerekenler](#hid-7-6)

<br/>
<br/>

---

<h1 id="hid-1">1. Önsöz - Amaç</h1>

Bu el kitabı, ROS2 altyapısı kullanılarak otonom mobil robot (AMR) geliştirme sürecini profesyonel ve sistematik bir şekilde öğrenmek isteyen mühendisler, öğrenciler ve araştırmacılar için hazırlanmıştır.

### Bu Dokümanın Hedefleri

* AMR geliştirirken ihtiyaç duyulan temel teorik kavramları aktarmak
* ROS2 ekosisteminin AMR'lerle ilişkili bileşenlerini ayrıntılı biçimde tanıtmak
* Uygulamaya yönelik, düzenli ve tek kaynaktan takip edilebilir bir referans sunmak
* Simülasyonda bir AMR'yi tasarlamak ve test etmek
* Gerçek robot üzerinde kullanılabilecek sağlam bir bilgi temeli oluşturmak

### Sistem Mimarisi Genel Bakış

```
┌─────────────────────────────────────────────────────────────────┐
│                         ÜST SİSTEM                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
│  │ slam_    │  │  Nav2    │  │  RViz2   │  │ robot_state_pub  │ │
│  │ toolbox  │  │          │  │          │  │                  │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘ │
│       │             │             │                 │           │
│       └─────────────┴─────────────┴─────────────────┘           │
│                           │                                     │
│                    ROS2 Topics/TF                               │
│                           │                                     │
├───────────────────────────┼─────────────────────────────────────┤
│                           │                                     │
│  ┌────────────────────────┴────────────────────────────────┐    │
│  │                    ros_gz_bridge                        │    │
│  └────────────────────────┬────────────────────────────────┘    │
│                           │                                     │
│                     ALT SİSTEM                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │   Gazebo     │  │   Motorlar   │  │   Sensörler          │   │
│  │   Sim        │  │   (DiffDrive)│  │   (LIDAR, IMU, vs.)  │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

<br/>
<br/>

---

<h1 id="hid-2">2. Konumlama</h1>

Konumlama (*localization*), bir robotun uzayda hangi pozisyonda ve hangi yönelimde bulunduğunun ölçülmesi ve hesaplanması sürecidir. AMR sistemleri için konumlama, robotun çevresini anlaması ve güvenli şekilde hareket edebilmesi adına kritik bir bileşendir.

### Konumlama Neden Önemli?

Konumlama doğru yapılmadığında:
* Robot harita oluşturamaz
* Var olan harita üzerinde kendi konumunu bilemez
* Navigasyon ve çarpışma önleme mekanizmaları düzgün çalışamaz

ROS2, bu ihtiyacı karşılamak için `tf2` ismindeki güçlü koordinat sistemi yönetim altyapısını sağlar.

<h2 id="hid-2-1">2.1. Temel Kavramlar</h2>

Konumlama sisteminin temelinde iki önemli kavram bulunur: **frame** ve **transform**.

### Frame (Çerçeve)

Bir frame, uzayda bir referans noktasıdır. Her sensör, her robot parçası ve robotun kendisi bir frame ile temsil edilir.

### Transform (Dönüşüm)

Transform, iki frame arasındaki göreli konum ve yönelim farkını ifade eder. Yani:

> **"A frame'ine göre baktığımızda B frame'i nerededir?"** sorusunun cevabıdır.

Bir transform konumlama olarak şu iki bilgiyi içerir:

| Bileşen | Değerler | Açıklama |
|---------|----------|----------|
| **Pozisyon** | x, y, z | Metre cinsinden konum |
| **Oryantasyon** | x, y, z, w | Quaternion formatında yönelim |

### Transform İlişkisi

```
B = A + T_{A→B}
```

Burada **T_{A→B}**, A'dan B'ye dönüşümü ifade eder.

<h2 id="hid-2-3">2.3. Frame Ağacı (TF Tree)</h2>

ROS2'de tüm frame'ler hiyerarşik bir ağaç yapısı oluşturur. Bu yapıya **TF Tree** denir.

### Tipik AMR TF Ağacı

```
                    map
                     │
                     │ (AMCL/SLAM tarafından güncellenir)
                     ▼
                    odom
                     │
                     │ (Odometri tarafından güncellenir)
                     ▼
               base_footprint
                     │
                     │ (Sabit - robot modelinden)
                     ▼
                 base_link
                 /   │   \
                /    │    \
               ▼     ▼     ▼
           wh_l   chasis   wh_r
                     │
                     ▼
                   lidar
```

### TF Kuralları

Bir frame'in:
* **yalnızca bir ebeveyni** olabilir
* **birden fazla çocuğu** olabilir

Bu sayede herhangi iki frame arasındaki konum, doğrudan bağlı olmasalar bile hesaplanabilir.

<br/>
<br/>

---

<h1 id="hid-3">3. Modelleme (URDF)</h1>

Robotu simülasyon ortamında veya gerçek dünyada çalıştırabilmek için önce robotun fiziksel yapısının tanımlanması gerekir. ROS2'de robot modelleme için **URDF (Unified Robot Description Format)** kullanılır.

URDF, XML tabanlı bir format olup bir robotun:
* Geometrisini
* Eklemlerini
* Kütlesini ve atalet özelliklerini
* Çarpışma modellerini
* Görsel temsilini

ayrıntılı bir şekilde tanımlamaya imkân verir.

### URDF → TF Dönüşümü

URDF modeli oluşturulduğunda, ROS2 bu modeli otomatik olarak TF yapısına dönüştürür:
* Her **link** → bir **frame** olarak
* Her **joint** → bir **transform** olarak temsil edilir

<h2 id="hid-3-1">3.1. Temel Kavramlar (Link ve Joint)</h2>

URDF iki ana yapı üzerine kuruludur:

### Link

Robotun fiziksel parçalarını temsil eder. Gövde, tekerlek, kol segmenti gibi her fiziksel yapı bir link'tir.

### Joint

İki link arasındaki mekanik bağı ve hareket ilişkisini tanımlar.

```
┌─────────────────────────────────────────────────────────┐
│                         URDF                            │
│                                                         │
│    ┌──────┐         ┌──────┐         ┌──────┐          │
│    │Link A│─────────│Joint │─────────│Link B│          │
│    └──────┘  parent └──────┘  child  └──────┘          │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

<h2 id="hid-3-2">3.2. Link Anlatısı</h2>

Bir link, üç temel bileşenden oluşur:

### 1. Visual Tag

`<visual>` etiketi, bir link'in simülasyon ortamında nasıl görüneceğini tanımlar.

```xml
<visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <box size="1 0.5 0.2" />
        <!-- veya -->
        <cylinder radius="0.2" length="1" />
        <!-- veya -->
        <sphere radius="0.1" />
        <!-- veya -->
        <mesh filename="package://pkg_name/meshes/model.dae" scale="1 1 1" />
    </geometry>
    <material name="gray">
        <color rgba="0.5 0.5 0.5 1" />
    </material>
</visual>
```

### 2. Collision Tag

`<collision>` etiketi, fizik motorunun çarpışma hesaplamaları için kullandığı modeli tanımlar.

```xml
<collision>
    <origin xyz="0 0 0.05" rpy="0 0 0" />
    <geometry>
        <box size="1 0.5 0.2" />
    </geometry>
</collision>
```

> **Not:** Collision geometrisi mümkün olduğunca basit tutulmalıdır (box, sphere, cylinder).

### 3. Inertial Tag

`<inertial>` etiketi, link'in fiziksel davranışları için gerekli kütle ve atalet bilgilerini içerir.

```xml
<inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="1.0" />
    <inertia
        ixx="0.02" ixy="0.0" ixz="0.0"
        iyy="0.02" iyz="0.0"
        izz="0.03" />
</inertial>
```

### Atalet Formülleri (Yaygın Şekiller)

| Şekil | Ixx | Iyy | Izz |
|-------|-----|-----|-----|
| **Kutu** | m(h²+d²)/12 | m(w²+d²)/12 | m(w²+h²)/12 |
| **Silindir** | m(3r²+h²)/12 | m(3r²+h²)/12 | mr²/2 |
| **Küre** | 2mr²/5 | 2mr²/5 | 2mr²/5 |

<h2 id="hid-3-3">3.3. Joint Anlatısı</h2>

URDF, dört temel joint türü sunar:

### Joint Türleri Karşılaştırması

| Tür | Hareket | limit Gerekir mi? | Kullanım Alanı |
|-----|---------|-------------------|----------------|
| **fixed** | Yok | Hayır | Sensör montajı, şasi |
| **revolute** | Sınırlı dönüş | Evet | Robot kolu |
| **continuous** | Sınırsız dönüş | Hayır | Tekerlek |
| **prismatic** | Doğrusal | Evet | Lineer aktüatör |

### Fixed Joint Örneği

```xml
<joint name="lidar_mount" type="fixed">
    <parent link="base_link" />
    <child link="lidar_link" />
    <origin xyz="0 0 0.2" rpy="0 0 0" />
</joint>
```

### Continuous Joint Örneği (Tekerlek)

```xml
<joint name="left_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="left_wheel" />
    <origin xyz="0 0.25 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
</joint>
```

### Revolute Joint Örneği

```xml
<joint name="arm_joint" type="revolute">
    <parent link="arm_base" />
    <child link="arm_link" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="5" velocity="1.0" />
</joint>
```

<h2 id="hid-3-5">3.5. URDF → ROS2 Bağlantısı</h2>

URDF dosyasının ROS2 sistemi tarafından kullanılmasını sağlayan temel mekanizma `robot_state_publisher` düğümüdür.

### Çalışma Akışı

```
┌─────────────────┐     ┌─────────────────────┐     ┌──────────────┐
│  URDF/Xacro     │────▶│ robot_state_        │────▶│ /tf          │
│  Dosyası        │     │ publisher           │     │ /tf_static   │
└─────────────────┘     └─────────────────────┘     └──────────────┘
                               ▲
                               │
                        ┌──────┴──────┐
                        │/joint_states│
                        └─────────────┘
```

### robot_state_publisher'ın Görevleri

1. **URDF Okuma:** Kinematik yapıyı yükler
2. **TF Güncelleme:** `/joint_states`'e göre transform'ları günceller
3. **TF Yayınlama:** `map → odom → base_link → ...` zincirini yayınlar

<h2 id="hid-3-7">3.7. Xacro Aracı</h2>

**Xacro (XML Macros)**, URDF'i makrolar ve değişkenler aracılığıyla modülerleştiren bir ön işlemcidir.

### Xacro Özellikleri

| Özellik | Açıklama |
|---------|----------|
| `<xacro:property>` | Değişken tanımlama |
| `<xacro:macro>` | Tekrarlayan blokları fonksiyon gibi kullanma |
| `<xacro:include>` | Dosya dahil etme |
| `${...}` | Matematiksel ifade hesaplama |
| `<xacro:if>` | Koşullu içerik |

### Xacro Örneği - Tekerlek Macro

```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_width" value="0.04"/>

<xacro:macro name="wheel" params="name x_pos y_pos">
    <link name="${name}_wheel">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
            </geometry>
        </visual>
    </link>
    
    <joint name="${name}_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="${name}_wheel"/>
        <origin xyz="${x_pos} ${y_pos} 0" rpy="${pi/2} 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
</xacro:macro>

<!-- Kullanım -->
<xacro:wheel name="left" x_pos="0" y_pos="0.2"/>
<xacro:wheel name="right" x_pos="0" y_pos="-0.2"/>
```

<br/>
<br/>

---

<h1 id="hid-4">4. Simülasyon (Gazebo)</h1>

Simülasyon, gerçek dünyadaki olayların bilgisayar ortamında taklit edilmesidir. Bu eğitimde **Gazebo Sim** (yeni nesil Gazebo) kullanılmaktadır.

### Gazebo Classic vs Gazebo Sim

| Özellik | Gazebo Classic | Gazebo Sim |
|---------|----------------|------------|
| Durum | Eski | Yeni (önerilen) |
| ROS2 Entegrasyonu | ros_ign_* | ros_gz_* |
| Fizik Motoru | ODE | DART, Bullet, vs. |

### Kurulum (Ubuntu 24.04 + ROS2 Jazzy)

```bash
# Gazebo Sim kurulumu
sudo apt install gz-sim

# ROS-Gazebo entegrasyon paketleri
sudo apt install ros-${ROS_DISTRO}-ros-gz
```

<h2 id="hid-4-2">4.2. SDF - URDF Etkileşimi</h2>

Gazebo, URDF içindeki `<gazebo>` etiketlerini okur ve SDF'e dönüştürür.

### Plugin Ekleme

```xml
<gazebo>
    <plugin 
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
        <topic>cmd_vel</topic>
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
    </plugin>
</gazebo>
```

### Link Özelliklerini Değiştirme

```xml
<gazebo reference="caster_wheel">
    <collision>
        <surface>
            <friction>
                <ode>
                    <mu>0.001</mu>
                    <mu2>0.001</mu2>
                </ode>
            </friction>
        </surface>
    </collision>
</gazebo>
```

<h2 id="hid-4-3">4.3. Gazebo - ROS Bağlantısı</h2>

Gazebo Sim, ROS2 ile aynı iletişim sistemini kullanmaz. Köprü gereklidir.

### Paketler

| Paket | Amaç |
|-------|------|
| `ros_gz_sim` | Gazebo başlatma, robot spawn |
| `ros_gz_bridge` | Topic köprüleme |

### ros_gz_bridge YAML Yapısı

```yaml
# gz_bridge.yaml
- ros_topic_name: "cmd_vel"
  gz_topic_name: "robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "joint_states"
  gz_topic_name: "robot/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "lidar/scan"
  gz_topic_name: "lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

<h2 id="hid-4-4">4.4. Pluginler</h2>

### AMR İçin Temel Pluginler

| Plugin | Dosya Adı | Amaç |
|--------|-----------|------|
| Joint State Publisher | `gz-sim-joint-state-publisher-system` | Eklem durumlarını yayınlar |
| Differential Drive | `gz-sim-diff-drive-system` | Diferansiyel sürüş |
| Odometry Publisher | `gz-sim-odometry-publisher-system` | Odometri verisi |
| Sensors | `gz-sim-sensors-system` | Sensör altyapısı |

### Differential Drive Plugin Örneği

```xml
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
    <topic>cmd_vel</topic>
    <left_joint>base_TO_wh_l</left_joint>
    <right_joint>base_TO_wh_r</right_joint>
    <wheel_separation>0.44</wheel_separation>
    <wheel_radius>0.1</wheel_radius>
    <max_linear_acceleration>1.0</max_linear_acceleration>
    <max_angular_acceleration>2.0</max_angular_acceleration>
</plugin>
```

<h2 id="hid-4-5">4.5. Sensörler</h2>

### LIDAR Sensör Tanımı

```xml
<gazebo reference="lidar">
    <sensor name="gpu_lidar" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose>
        <topic>lidar/scan</topic>
        <update_rate>10</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>640</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14</min_angle>
                    <max_angle>3.14</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.08</min>
                <max>15.0</max>
                <resolution>0.01</resolution>
            </range>
        </ray>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <gz_frame_id>lidar</gz_frame_id>
    </sensor>
</gazebo>
```

> **Önemli:** Sensörlerin çalışması için dünya dosyasında `gz-sim-sensors-system` plugin'i olmalıdır.

<br/>
<br/>

---

<h1 id="hid-5">5. Haritalama (slam_toolbox)</h1>

**slam_toolbox**, ROS2 ekosistemi için geliştirilmiş modern bir 2D SLAM paketidir.

### SLAM Nedir?

**S**imultaneous **L**ocalization **A**nd **M**apping

* Robot nerede? → Localization
* Çevre nasıl? → Mapping

Bu iki soruyu aynı anda çözmeye çalışır.

<h2 id="hid-5-1">5.1. slam_toolbox Çalışma Prensibi</h2>

### Veri Akışı

```
┌──────────────┐     ┌──────────────────┐     ┌─────────────┐
│ /scan        │────▶│                  │────▶│ /map        │
│ (LaserScan)  │     │   slam_toolbox   │     │ (OccGrid)   │
└──────────────┘     │                  │     └─────────────┘
                     │                  │
┌──────────────┐     │                  │     ┌─────────────┐
│ /tf          │────▶│                  │────▶│ map→odom TF │
│ (odom→base)  │     └──────────────────┘     └─────────────┘
└──────────────┘
```

### Temel Kavramlar

| Kavram | Açıklama |
|--------|----------|
| **Pose Graph** | Robotun pozlarını düğüm olarak tutan graf yapısı |
| **Scan Matching** | LIDAR verilerini haritayla eşleştirme |
| **Loop Closure** | Daha önce geçilen yere dönünce hata düzeltme |
| **Occupancy Grid** | Hücre tabanlı doluluk haritası |

<h2 id="hid-5-2">5.2. Modlar</h2>

### Mapping vs Localization

| Mod | Amaç | Harita |
|-----|------|--------|
| `mapping` | Yeni harita oluşturma | Dinamik |
| `localization` | Var olan haritada konum bulma | Sabit |

### Sync vs Async

| Mod | Özellik | Öneri |
|-----|---------|-------|
| `sync` | Zaman uyumlu | Gerçek robot |
| `async` | Zaman bağımsız | Yüksek frekanslı LIDAR |

<h2 id="hid-5-3">5.3. Kritik Parametreler</h2>

```yaml
slam_toolbox:
  ros__parameters:
    # Frame Tanımları
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    
    # Mod
    mode: mapping  # veya localization
    
    # Harita
    resolution: 0.05
    max_laser_range: 12.0
    
    # Hareket Eşikleri
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    
    # Loop Closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    
    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
```

<h2 id="hid-5-4">5.4. Kullanım</h2>

### Haritalama Başlatma

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### Harita Kaydetme

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

Çıktı:
* `my_map.pgm` - Harita görüntüsü
* `my_map.yaml` - Meta veri

<br/>
<br/>

---

<h1 id="hid-6">6. Navigasyon (nav2)</h1>

**Nav2 (Navigation2)**, ROS2 tabanlı mobil robotların konumunu bulması, hedefe güvenli şekilde gitmesi ve engellerden kaçınması için kullanılan framework'tür.

### Nav2 Temel Soruları

1. **Ben neredeyim?** → Localization (AMCL)
2. **Nereye gideceğim?** → Goal
3. **Oraya nasıl gideceğim?** → Path Planning
4. **Engel çıkarsa ne yapacağım?** → Obstacle Avoidance

<h2 id="hid-6-1">6.1. Nav2 Mimarisi</h2>

```
┌─────────────────────────────────────────────────────────────┐
│                      Behavior Tree                          │
│                      (bt_navigator)                         │
└─────────────────────────┬───────────────────────────────────┘
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
    ┌──────────┐    ┌──────────┐    ┌──────────┐
    │ Planner  │    │Controller│    │ Recovery │
    │ Server   │    │  Server  │    │ Behaviors│
    └────┬─────┘    └────┬─────┘    └──────────┘
         │               │
         ▼               ▼
    ┌──────────┐    ┌──────────┐
    │ Global   │    │  Local   │
    │ Costmap  │    │ Costmap  │
    └──────────┘    └──────────┘
```

<h2 id="hid-6-2">6.2. AMCL Detaylı Anlatım</h2>

**AMCL (Adaptive Monte Carlo Localization)**, parçacık filtresi kullanarak robotun haritadaki konumunu tahmin eder.

### Çalışma Prensibi

1. **Particle Dağıtımı:** Olası pozisyonlar haritaya serpiştirilir
2. **Hareket Modeli:** Robot hareket edince particle'lar da hareket eder
3. **Sensör Modeli:** LIDAR ile harita karşılaştırılır, iyi eşleşenler ağırlık kazanır
4. **Resampling:** Kötü particle'lar elenir, iyiler çoğaltılır

### AMCL Parametreleri

```yaml
amcl:
  ros__parameters:
    # Particle Sayısı
    min_particles: 500
    max_particles: 2000
    
    # Frame'ler
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom"
    
    # Sensör
    scan_topic: /scan
    laser_max_range: 12.0
    
    # Hareket Modeli (Odometri hatası)
    alpha1: 0.2  # Dönüşten dönüşe
    alpha2: 0.2  # Hareketten dönüşe
    alpha3: 0.2  # Hareketten harekete
    alpha4: 0.2  # Dönüşten harekete
```

<h2 id="hid-6-3">6.3. Costmap Yapısı</h2>

### Global vs Local Costmap

| Costmap | Kapsam | Güncelleme | Kullanım |
|---------|--------|------------|----------|
| **Global** | Tüm harita | Yavaş | Yol planlama |
| **Local** | Robot çevresi | Hızlı | Engel kaçınma |

### Costmap Layers

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

<h2 id="hid-6-4">6.4. Planner ve Controller</h2>

### Planner Seçenekleri

| Planner | Açıklama |
|---------|----------|
| NavFn | Klasik A* tabanlı |
| Smac Planner | Modern, hybrid A* |
| Theta* | Daha düzgün yollar |

### Controller Seçenekleri

| Controller | Açıklama |
|------------|----------|
| DWB | Dynamic Window |
| Regulated Pure Pursuit | Düzgün takip |
| MPPI | Model Predictive |

<h2 id="hid-6-5">6.5. Nav2 Parametreleri</h2>

Tam parametre dosyası için: [resources/configs/nav2_params.yaml](./resources/configs/nav2_params.yaml)

<br/>
<br/>

---

<h1 id="hid-7">7. Simülasyon vs Gerçeklik</h1>

Simülasyonda çalışan sistem, gerçek robota taşındığında farklı davranabilir.

<h2 id="hid-7-1">7.1. Benzerlik ve Farklılıklar</h2>

### Değişmeyen Kısımlar
* URDF modeli
* slam_toolbox ve Nav2 parametreleri (çoğu)
* ROS2 topic/service yapısı

### Değişen Kısımlar

| Simülasyon | Gerçek |
|------------|--------|
| Gazebo plugin'ler hareketi sağlar | Motor sürücü + mikrodenetleyici |
| Sensörler otomatik | Sensör sürücüleri gerekli |
| use_sim_time: true | use_sim_time: false |

<h2 id="hid-7-2">7.2. Temel Sistem Tasarımı</h2>

```
┌─────────────────────────────────────────────────────────────┐
│                       ÜST SİSTEM                            │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────────┐ │
│  │ slam_      │  │   Nav2     │  │ Odometri Hesaplama     │ │
│  │ toolbox    │  │            │  │ (tekerlek → odom msg)  │ │
│  └────────────┘  └────────────┘  └────────────────────────┘ │
│                                                             │
│  ┌────────────────────────────────────────────────────────┐ │
│  │              LiDAR / Sensör Sürücüleri                 │ │
│  └────────────────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────┤
│                       ALT SİSTEM                            │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  micro-ROS / Seri Haberleşme                           │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────────┐ │
│  │ DiffDrive  │  │ Encoder    │  │ Motor Sürücü           │ │
│  │ Hesaplama  │  │ Okuma      │  │ (PWM + PID)            │ │
│  └────────────┘  └────────────┘  └────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

<h2 id="hid-7-3">7.3. Odometri Hesaplaması</h2>

Diferansiyel sürüşte odometri, tekerlek hareketlerinden konum tahmini yapar.

### Değişkenler

| Sembol | Açıklama |
|--------|----------|
| $d_l$ | Sol tekerin aldığı yol (m) |
| $d_r$ | Sağ tekerin aldığı yol (m) |
| $L$ | İki teker arası mesafe (m) |
| $x_0, y_0, \theta_0$ | Başlangıç konumu ve açısı |
| $x_1, y_1, \theta_1$ | Yeni konum ve açı |

### Düz Hareket ($d_l \approx d_r$)

$$
x_1 = x_0 + \frac{d_l + d_r}{2} \cdot \cos(\theta_0)
$$

$$
y_1 = y_0 + \frac{d_l + d_r}{2} \cdot \sin(\theta_0)
$$

$$
\theta_1 = \theta_0
$$

### Eğik Hareket ($d_l \neq d_r$)

Yardımcı değişkenler:

$$
\alpha = \frac{d_r - d_l}{L}
$$

$$
r = \frac{L}{2} \cdot \frac{d_l + d_r}{d_r - d_l}
$$

Konum güncellemesi:

$$
x_1 = x_0 + r \cdot (\cos(\theta_0 + \alpha) - \cos(\theta_0))
$$

$$
y_1 = y_0 + r \cdot (\sin(\theta_0 + \alpha) - \sin(\theta_0))
$$

$$
\theta_1 = \theta_0 + \alpha
$$

### Python Implementasyonu

```python
import math

class DifferentialOdometry:
    def __init__(self, wheel_separation: float):
        self.L = wheel_separation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
    
    def update(self, dl: float, dr: float):
        """
        dl: Sol tekerlek mesafesi (m)
        dr: Sağ tekerlek mesafesi (m)
        """
        if abs(dl - dr) < 1e-6:
            # Düz hareket
            d = (dl + dr) / 2.0
            self.x += d * math.cos(self.theta)
            self.y += d * math.sin(self.theta)
        else:
            # Eğik hareket
            alpha = (dr - dl) / self.L
            r = (self.L / 2.0) * (dl + dr) / (dr - dl)
            
            self.x += r * (math.cos(self.theta + alpha) - math.cos(self.theta))
            self.y += r * (math.sin(self.theta + alpha) - math.sin(self.theta))
            self.theta += alpha
        
        # Açıyı [-π, π] aralığında tut
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return self.x, self.y, self.theta
```

<h2 id="hid-7-5">7.5. micro-ROS Kullanımı</h2>

micro-ROS, mikrodenetleyicilerde ROS2 benzeri programlama sağlar.

### Mimari

```
┌─────────────────┐         ┌─────────────────┐
│ Mikrodenetleyici│◄───────►│ micro-ROS Agent │◄───► ROS2 Ağı
│ (micro-ROS      │  UART/  │ (Linux'ta)      │
│  Client)        │  UDP    │                 │
└─────────────────┘         └─────────────────┘
```

### Örnek: cmd_vel Dinleme (Arduino)

```cpp
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

void subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = 
        (const geometry_msgs__msg__Twist *)msgin;
    
    float linear = msg->linear.x;
    float angular = msg->angular.z;
    
    // Diferansiyel hız hesaplama
    float v_left = linear - (angular * WHEEL_SEPARATION / 2.0);
    float v_right = linear + (angular * WHEEL_SEPARATION / 2.0);
    
    // Motor kontrolü
    setMotorSpeed(LEFT_MOTOR, v_left);
    setMotorSpeed(RIGHT_MOTOR, v_right);
}

void setup() {
    // micro-ROS başlatma
    set_microros_transports();
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);
    
    // Node oluşturma
    rcl_node_t node;
    rclc_node_init_default(&node, "motor_controller", "", &support);
    
    // Subscriber oluşturma
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
}
```

<h2 id="hid-7-6">7.6. Dikkat Edilmesi Gerekenler</h2>

### Kritik Kontrol Listesi

- [ ] `use_sim_time: false` ayarlandı mı?
- [ ] Frame isimleri tutarlı mı? (base_link, odom, map)
- [ ] LIDAR topic adı doğru mu?
- [ ] Tekerlek yarıçapı ve aralığı doğru girildi mi?
- [ ] Encoder çözünürlüğü hesaplandı mı?
- [ ] Motor yönleri doğru mu?
- [ ] Zaman damgaları senkronize mi?

### Yaygın Hatalar ve Çözümleri

| Hata | Olası Sebep | Çözüm |
|------|-------------|-------|
| Robot haritada kayıyor | Odometri hatası | Tekerlek parametrelerini kalibre et |
| SLAM harita bozuk | LIDAR frame yanlış | gz_frame_id kontrol et |
| Nav2 hedef reddediyor | Transform timeout | transform_tolerance artır |
| Robot titriyor | PID ayarsız | Controller kazançlarını düşür |

<br/>
<br/>

---

> **📝 Not:** Bu doküman [GARPamr_edu](https://github.com/ngen01/kayra_ros) projesinden esinlenerek geliştirilmiştir.
