<h1>Uygulamalar</h1>

> **📌 GARPamr_edu** - Tüm Çözümler Tamamlanmış Versiyon

---

**0. [Robotik Kol Modeli](#hid-0)**
**1. [Robot Modelinin Çıkarımı](#hid-1)**
**2. [Gazebo Launcher'ı Oluşturumu](#hid-2)**
**3. [Diferansiyel Sürüş Plugini Eklenmesi](#hid-3)**
**4. [Klavye Kontrol (teleop keyboard)](#hid-4)**
**5. [Lidar Sensörünün Eklenmesi](#hid-5)**
**6. [Odometri Plugini ve ROS2 Entegrasyonu](#hid-6)** ✅
**7. [Dünya Ve Model Ekleme](#hid-7)**
**8. [Ev Modeli ve Dünyası Oluşturma](#hid-8)**
**9. ['slam_toolbox' İle Haritalama](#hid-9)** ✅
**10. ['nav2' İle Navigasyon](#hid-10)** ✅
**11. ['nav2' İle Waypoint Navigasyonu](#hid-11)** ✅

---

<h1 id="hid-0">0. Robotik Kol Modeli</h1>

#### Adım 1: Temel Robot Kol

Aşağıdaki gibi bir robot kol modeli oluşturun:
- Rail (mavi kutu)
- Slider (kırmızı kutu, prismatic joint)
- First arm (yeşil silindir)
- Second arm (kırmızı silindir, revolute joint)

<details>
<summary>Çözümü görmek için tıklayın</summary>

```xml
<?xml version="1.0"?>
<robot name="robot_kol">

    <material name="red">
        <color rgba="1 0 0 1" />
    </material>
    <material name="green">
        <color rgba="0 1 0 1" />
    </material>
    <material name="blue">
        <color rgba="0 0 1 1" />
    </material>

    <link name="base_link"/>

    <joint name="base_link_TO_rail" type="fixed">
        <parent link="base_link" />
        <child link="rail" />
        <origin xyz="0 0 0" rpy="0 0 0" />
    </joint>

    <link name="rail">
        <visual>
            <geometry>
                <box size="0.4 0.2 0.05" />
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="rail_TO_slider" type="prismatic">
        <parent link="rail" />
        <child link="slider" />
        <origin xyz="0 0 0.035" rpy="0 0 0" />
        <axis xyz="1 0 0" />
        <limit lower="-0.175" upper="0.175" effort="100" velocity="100" />
    </joint>

    <link name="slider">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.02" />
            </geometry>
            <material name="red"/>
        </visual>
    </link>

    <joint name="slider_TO_first_arm" type="fixed">
        <parent link="slider" />
        <child link="first_arm" />
        <origin xyz="0 0 0.12" rpy="0 0 0" />
    </joint>

    <link name="first_arm">
        <visual>
            <geometry>
                <cylinder radius="0.02" length="0.2" />
            </geometry>
            <material name="green"/>
        </visual>
    </link>

    <joint name="first_arm_TO_second_arm" type="revolute">
        <parent link="first_arm" />
        <child link="second_arm" />
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <axis xyz="0 1 0" />
        <limit lower="0.0" upper="1.57" effort="100" velocity="100" />
    </joint>

    <link name="second_arm">
        <visual>
            <origin xyz="0 0 0.1" rpy="0 0 0" />
            <geometry>
                <cylinder radius="0.02" length="0.2" />
            </geometry>
            <material name="red"/>
        </visual>
    </link>
</robot>
```
</details>

---

<h1 id="hid-1">1. Robot Modelinin Çıkarımı</h1>

Diferansiyel sürüşe uygun bir robot tasarlayın:
- Workspace oluşturun
- Robot modelini taşıyacak bir paket açın
- Modelinizi oluşturun
- Basit bir launch file oluşturun

<details>
<summary>Çözümü görmek için tıklayın</summary>

### Workspace ve Paket Oluşturma

```bash
mkdir -p garp_ws/src
cd garp_ws/src
ros2 pkg create garp_description
rm -rf garp_description/src garp_description/include
mkdir garp_description/urdf garp_description/launch
```

### CMakeLists.txt Düzenleme

```cmake
# ament_package() satırının üstüne ekleyin
install(
    DIRECTORY urdf launch
    DESTINATION share/${PROJECT_NAME}/
)
```

### main.urdf.xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="garp_robot">

    <!-- Renk Tanımlamaları -->
    <material name="green">
        <color rgba="0 1 0 1"/>
    </material>
    <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
    </material>

    <!-- Şase Nitelikleri -->
    <xacro:property name="chasis_len_x" value="0.6"/>
    <xacro:property name="chasis_len_y" value="0.4"/>
    <xacro:property name="chasis_len_z" value="0.2"/>
    <xacro:property name="base_chasis_offset" value="0.08"/>
    <xacro:property name="chasis_mass" value="2.0"/>

    <!-- Tekerlek Nitelikleri -->
    <xacro:property name="wh_radius" value="0.1"/>
    <xacro:property name="wh_thick" value="0.04"/>
    <xacro:property name="wh_chasis_gap" value="0.001"/>

    <!-- Dosya Dahil Etme -->
    <xacro:include filename="./wheels.urdf.xacro" />

    <!-- Base Links -->
    <link name="base_footprint"/>
    <link name="base_link"/>

    <joint name="base_TO_footprint" type="fixed">
        <origin xyz="0.0 0.0 ${wh_radius}" rpy="0.0 0.0 0.0"/>
        <parent link="base_footprint"/>
        <child link="base_link"/>
    </joint>

    <!-- Şase -->
    <joint name="base_TO_chasis" type="fixed">
        <origin xyz="0.0 0.0 ${base_chasis_offset}" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="chasis"/>
    </joint>

    <link name="chasis">
        <inertial>
            <mass value="${chasis_mass}"/>
            <inertia 
                ixx="${chasis_mass * (chasis_len_y*chasis_len_y + chasis_len_z*chasis_len_z) / 12.0}"
                iyy="${chasis_mass * (chasis_len_x*chasis_len_x + chasis_len_z*chasis_len_z) / 12.0}"
                izz="${chasis_mass * (chasis_len_x*chasis_len_x + chasis_len_y*chasis_len_y) / 12.0}"
                ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <visual>
            <geometry>
                <box size="${chasis_len_x} ${chasis_len_y} ${chasis_len_z}"/>
            </geometry>
            <material name="green"/>
        </visual>
        <collision>
            <geometry>
                <box size="${chasis_len_x} ${chasis_len_y} ${chasis_len_z}"/>
            </geometry>
        </collision>
    </link>

</robot>
```

### wheels.urdf.xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="garp_robot">

    <!-- Tahrik Tekerleği Makrosu -->
    <xacro:macro name="create_wh" params="postfix yfact">
        <joint name="base_TO_wh_${postfix}" type="continuous">
            <origin xyz="0.0 ${yfact * (chasis_len_y / 2.0 + wh_thick / 2.0 + wh_chasis_gap)} 0.0"/>
            <parent link="base_link"/>
            <child link="wh_${postfix}"/>
            <axis xyz="0.0 1.0 0.0"/>
        </joint>

        <link name="wh_${postfix}">
            <inertial>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
                <mass value="1.0"/>
                <inertia ixx="0.01" iyy="0.01" izz="0.005" ixy="0" ixz="0" iyz="0"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
                <geometry>
                    <cylinder radius="${wh_radius}" length="${wh_thick}"/>
                </geometry>
                <material name="gray"/>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0"/>
                <geometry>
                    <cylinder radius="${wh_radius}" length="${wh_thick}"/>
                </geometry>
            </collision>
        </link>
    </xacro:macro>

    <!-- Tekerlekleri Oluştur -->
    <xacro:create_wh postfix="l" yfact="1.0"/>
    <xacro:create_wh postfix="r" yfact="-1.0"/>

    <!-- Caster Wheel Makrosu -->
    <xacro:property name="cwh_radius" value="${(wh_radius - (chasis_len_z / 2.0 - base_chasis_offset)) / 2.0}"/>

    <xacro:macro name="create_cwh" params="postfix xfact yfact">
        <joint name="chasis_TO_cwh_${postfix}" type="fixed">
            <origin xyz="${xfact * (chasis_len_x / 2.0 - 0.1)} 
                        ${yfact * (chasis_len_y / 2.0 - 0.1)} 
                        ${-1 * (chasis_len_z / 2.0 + cwh_radius)}"/>
            <parent link="chasis"/>
            <child link="cwh_${postfix}"/>
        </joint>

        <link name="cwh_${postfix}">
            <inertial>
                <mass value="0.01"/>
                <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/>
            </inertial>
            <visual>
                <geometry>
                    <sphere radius="${cwh_radius}"/>
                </geometry>
                <material name="gray"/>
            </visual>
            <collision>
                <geometry>
                    <sphere radius="${cwh_radius}"/>
                </geometry>
            </collision>
        </link>

        <gazebo reference="cwh_${postfix}">
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
    </xacro:macro>

    <xacro:create_cwh postfix="lf" xfact="1.0" yfact="1.0"/>
    <xacro:create_cwh postfix="lb" xfact="-1.0" yfact="1.0"/>
    <xacro:create_cwh postfix="rf" xfact="1.0" yfact="-1.0"/>
    <xacro:create_cwh postfix="rb" xfact="-1.0" yfact="-1.0"/>

</robot>
```

### display.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory("garp_description")
    urdf_path = os.path.join(pkg_share, "urdf", "main.urdf.xacro")
    robot_desc = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
        Node(
            package="rviz2",
            executable="rviz2"
        ),
    ])
```

### Build ve Çalıştırma

```bash
cd garp_ws
colcon build
source install/setup.bash
ros2 launch garp_description display.launch.py
```

</details>

---

<h1 id="hid-2">2. Gazebo Launcher'ı Oluşturumu</h1>

Gazebo'yu başlatacak ve robotu spawn edecek bir launcher ayarlayın.

<details>
<summary>Çözümü görmek için tıklayın</summary>

### g_plugins.urdf.xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="garp_robot">

    <gazebo>
        <plugin 
            filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
            <topic>garp/joint_states</topic>
        </plugin>
    </gazebo>

</robot>
```

### gz_bridge.yaml

```yaml
- ros_topic_name: "joint_states"
  gz_topic_name: "garp/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

### gazebo.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory("garp_description")
    urdf_path = os.path.join(pkg_share, "urdf", "main.urdf.xacro")
    gz_bridge_cfg = os.path.join(pkg_share, "config", "gz_bridge.yaml")
    
    robot_desc = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    return LaunchDescription([
        # Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
        ),
        
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}]
        ),
        
        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            parameters=[{'topic': 'robot_description'}],
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': gz_bridge_cfg}],
        ),
        
        # RViz
        Node(
            package="rviz2",
            executable="rviz2"
        ),
    ])
```

</details>

---

<h1 id="hid-3">3. Diferansiyel Sürüş Plugini Eklenmesi</h1>

<details>
<summary>Çözümü görmek için tıklayın</summary>

### g_plugins.urdf.xacro'ya Ekleme

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <topic>garp/cmd_vel</topic>
    <left_joint>base_TO_wh_l</left_joint>
    <right_joint>base_TO_wh_r</right_joint>
    <wheel_separation>${(chasis_len_y / 2.0 + wh_thick / 2.0 + wh_chasis_gap) * 2}</wheel_separation>
    <wheel_radius>${wh_radius}</wheel_radius>
</plugin>
```

### gz_bridge.yaml'a Ekleme

```yaml
- ros_topic_name: "cmd_vel"
  gz_topic_name: "garp/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

### Test

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.3}}"
```

</details>

---

<h1 id="hid-4">4. Klavye Kontrol (teleop)</h1>

<details>
<summary>Çözümü görmek için tıklayın</summary>

### teleop_keyboard.py

```python
#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
WASD Teleop
-----------------
  w : ileri
  s : geri
  a : sola dön
  d : sağa dön
  boşluk : dur
  q : çıkış
"""

def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

class TeleopWASD(Node):
    def __init__(self):
        super().__init__("teleop_wasd")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.declare_parameter("lin_speed", 0.5)
        self.declare_parameter("ang_speed", 0.5)
        
        self.lin_speed = self.get_parameter("lin_speed").value
        self.ang_speed = self.get_parameter("ang_speed").value
        
        self.twist = Twist()
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info(HELP)

    def loop(self):
        key = get_key(0.01)
        if key:
            key = key.lower()
            if key == "w":
                self.twist.linear.x = self.lin_speed
                self.twist.angular.z = 0.0
            elif key == "s":
                self.twist.linear.x = -self.lin_speed
                self.twist.angular.z = 0.0
            elif key == "a":
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.ang_speed
            elif key == "d":
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.ang_speed
            elif key == " ":
                self.twist = Twist()
            elif key == "q":
                self.pub.publish(Twist())
                rclpy.shutdown()
        
        self.pub.publish(self.twist)

def main():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    rclpy.init()
    node = TeleopWASD()
    
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
```

</details>

---

<h1 id="hid-5">5. LiDAR Sensörünün Eklenmesi</h1>

<details>
<summary>Çözümü görmek için tıklayın</summary>

### sensor_extensions.urdf.xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="garp_robot">

    <xacro:property name="lidar_radius" value="0.05" />
    <xacro:property name="lidar_length" value="0.02" />

    <link name="lidar">
        <visual>
            <geometry>
                <cylinder radius="${lidar_radius}" length="${lidar_length}"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${lidar_radius}" length="${lidar_length}"/>
            </geometry>
        </collision>
    </link>

    <joint name="chasis_TO_lidar" type="fixed">
        <parent link="chasis" />
        <child link="lidar" />
        <origin xyz="0 0 ${chasis_len_z / 2.0 + lidar_length / 2.0}" rpy="0 0 0" />
    </joint>

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

</robot>
```

### gz_bridge.yaml'a Ekleme

```yaml
- ros_topic_name: "lidar/scan"
  gz_topic_name: "lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

> **Not:** `gz-sim-sensors-system` plugin'i dünya dosyasına veya URDF'e eklenmelidir.

</details>

---

<h1 id="hid-6">6. Odometri Plugini ve ROS2 Entegrasyonu</h1>

✅ **Bu bölüm tamamlandı!**

<details>
<summary>Çözümü görmek için tıklayın</summary>

### g_plugins.urdf.xacro'ya Odometry Plugin Ekleme

```xml
<plugin
    filename="gz-sim-odometry-publisher-system"
    name="gz::sim::systems::OdometryPublisher">
    <odom_topic>garp/odom</odom_topic>
    <odom_frame>odom_frame</odom_frame>
    <robot_base_frame>base_footprint</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

### gz_bridge.yaml'a Ekleme

```yaml
- ros_topic_name: "odom"
  gz_topic_name: "garp/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

### odom_to_tf.py - Odometry'den TF Yayınlama

```python
#!/usr/bin/env python3
"""
Odometry mesajlarını TF transformlarına dönüştüren düğüm.
odom -> base_footprint transform'unu yayınlar.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Parametreler
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom_frame')
        self.declare_parameter('base_frame', 'base_footprint')
        
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry subscriber
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.get_logger().info(
            f'odom_to_tf başlatıldı: {odom_topic} -> {self.odom_frame} -> {self.base_frame}'
        )

    def odom_callback(self, msg: Odometry):
        # Transform mesajı oluştur
        t = TransformStamped()
        
        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Pozisyon
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Oryantasyon
        t.transform.rotation = msg.pose.pose.orientation
        
        # TF yayınla
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### setup.py'a Ekleme

```python
entry_points={
    'console_scripts': [
        "odom_to_tf = garp_scripts.odom_to_tf:main",
    ],
},
```

### Launch Dosyasına Ekleme

```python
Node(
    package='garp_scripts',
    executable='odom_to_tf',
    parameters=[{
        'odom_topic': 'odom',
        'odom_frame': 'odom_frame',
        'base_frame': 'base_footprint'
    }]
),
```

</details>

---

<h1 id="hid-7">7. Dünya Ve Model Ekleme</h1>

<details>
<summary>Çözümü görmek için tıklayın</summary>

### garp_simulate Paketi Oluşturma

```bash
cd app_ws/src
ros2 pkg create garp_simulate
rm -rf garp_simulate/include garp_simulate/src
mkdir garp_simulate/launch garp_simulate/config 
mkdir garp_simulate/worlds garp_simulate/models
```

### my_model/model.config

```xml
<?xml version="1.0" ?>
<model>
    <name>my_model</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name>Developer</name>
    </author>
    <description>Basit kutu model</description>
</model>
```

### my_model/model.sdf

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="my_model">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>1 1 1</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>1 1 1</size></box></geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

</details>

---

<h1 id="hid-8">8. Ev Modeli ve Dünyası Oluşturma</h1>

<details>
<summary>Çözümü görmek için tıklayın</summary>

Detaylı ev modeli için garp_ws/src/garp_simulate/models/my_house klasörüne bakın.

</details>

---

<h1 id="hid-9">9. 'slam_toolbox' İle Haritalama</h1>

✅ **Bu bölüm tamamlandı!**

<details>
<summary>Çözümü görmek için tıklayın</summary>

### stb_mapping.yaml

```yaml
slam_toolbox:
  ros__parameters:
    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom_frame
    map_frame: map
    base_frame: base_footprint
    scan_topic: /lidar/scan
    mode: mapping

    # Map Settings
    resolution: 0.05
    max_laser_range: 12.0
    map_update_interval: 3.0
    
    # Transform
    transform_publish_period: 0.02
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    # Movement Thresholds
    minimum_time_interval: 0.5
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    
    # Scan Matching
    use_scan_matching: true
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    
    # Loop Closure
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
```

### slam.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("garp_simulate")
    slam_params = os.path.join(pkg_share, "config", "stb_mapping.yaml")

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),
    ])
```

### Kullanım

```bash
# 1. Simülasyonu başlat
ros2 launch garp_simulate in_my_house.launch.py

# 2. SLAM başlat
ros2 launch garp_simulate slam.launch.py

# 3. Teleop ile sür
ros2 run garp_scripts teleop_keyboard

# 4. Haritayı kaydet
ros2 run nav2_map_server map_saver_cli -f my_map
```

</details>

---

<h1 id="hid-10">10. 'nav2' İle Navigasyon</h1>

✅ **Bu bölüm tamamlandı!**

<details>
<summary>Çözümü görmek için tıklayın</summary>

### nav2_params.yaml (Özet)

Tam dosya için: [resources/configs/nav2_params.yaml](./resources/configs/nav2_params.yaml)

```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    odom_frame_id: "odom_frame"
    scan_topic: /lidar/scan
    min_particles: 500
    max_particles: 2000

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_theta: 1.0

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"

local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      robot_radius: 0.22

global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### navigation.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sim_pkg = get_package_share_directory("garp_simulate")
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")
    
    nav2_params = os.path.join(sim_pkg, "config", "nav2_params.yaml")
    
    map_yaml = LaunchConfiguration("map")
    
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(sim_pkg, "maps", "my_map.yaml"),
        description="Full path to map yaml file"
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": nav2_params,
            "use_sim_time": "true"
        }.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(sim_pkg, "config", "navigation.rviz")]
    )

    return LaunchDescription([
        declare_map,
        nav2_bringup,
        rviz,
    ])
```

### Kullanım

```bash
# 1. Simülasyonu başlat
ros2 launch garp_simulate in_my_house.launch.py

# 2. Navigasyonu başlat
ros2 launch garp_simulate navigation.launch.py map:=/path/to/my_map.yaml

# 3. RViz'de "2D Goal Pose" ile hedef belirle
```

</details>

---

<h1 id="hid-11">11. 'nav2' İle Waypoint Navigasyonu</h1>

✅ **Bu bölüm tamamlandı!**

<details>
<summary>Çözümü görmek için tıklayın</summary>

### waypoint_navigator.py

```python
#!/usr/bin/env python3
"""
Nav2 kullanarak sıralı waypoint'lere navigasyon yapan düğüm.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Nav2 Action Client
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        # Waypoint listesi (x, y, yaw)
        self.waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, -1.57),
        ]
        
        self.current_waypoint = 0
        
        self.get_logger().info('Waypoint Navigator başlatıldı')
        self.get_logger().info(f'Toplam {len(self.waypoints)} waypoint')
        
        # Başlat
        self.timer = self.create_timer(2.0, self.start_navigation)

    def start_navigation(self):
        self.timer.cancel()
        self.navigate_to_next()

    def navigate_to_next(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Tüm waypoint\'ler tamamlandı!')
            return
        
        wp = self.waypoints[self.current_waypoint]
        self.get_logger().info(
            f'Waypoint {self.current_waypoint + 1}/{len(self.waypoints)}: '
            f'({wp[0]}, {wp[1]}, {wp[2]})'
        )
        
        # Pose oluştur
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(wp[0], wp[1], wp[2])
        
        # Action server'ı bekle
        self._action_client.wait_for_server()
        
        # Goal gönder
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal reddedildi!')
            return
        
        self.get_logger().info('Goal kabul edildi')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoint {self.current_waypoint + 1} tamamlandı')
        
        self.current_waypoint += 1
        self.navigate_to_next()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        if remaining > 0:
            self.get_logger().info(f'Kalan mesafe: {remaining:.2f}m', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Kullanım

```bash
# 1. Simülasyon ve navigasyonu başlat
ros2 launch garp_simulate navigation.launch.py

# 2. Waypoint navigator'ı çalıştır
ros2 run garp_scripts waypoint_navigator
```

### Özelleştirme

Waypoint'leri değiştirmek için `self.waypoints` listesini düzenleyin:

```python
self.waypoints = [
    (x1, y1, yaw1),  # İlk hedef
    (x2, y2, yaw2),  # İkinci hedef
    # ...
]
```

</details>

---

> **📝 Not:** Bu doküman [GARPamr_edu](https://github.com/ngen01/kayra_ros) projesinden esinlenerek geliştirilmiştir.
