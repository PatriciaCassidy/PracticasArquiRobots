**PRÁCTICA 1**  
Análisis del sistema con ros2li  
1.Listar nodos en ejecución.  
<img width="589" height="178" alt="imagen" src="https://github.com/user-attachments/assets/4a2cab54-7fbb-437e-a9c6-4d193f6eae11" />  

2.Identificar topics relevantes (sensores, odometría, comandos).  
/clicked_point  
/clock  
/cmd_vel  
/events/bumper  
/goal_pose  
/initialpose  
/joint_states  
/odom  
/parameter_events  
/rgbd_camera/camera_info  
/rgbd_camera/depth_image  
/rgbd_camera/depth_image/compressed  
/rgbd_camera/depth_image/compressedDepth  
/rgbd_camera/depth_image/theora  
/rgbd_camera/depth_image/zstd  
/rgbd_camera/image  
/rgbd_camera/image/compressed  
/rgbd_camera/image/compressedDepth  
/rgbd_camera/image/theora  
/rgbd_camera/image/zstd  
/rgbd_camera/points  
/robot_description  
/rosout  
/scan_raw  
/tf  
/tf_static  

3.Inspeccionar el tipo de mensajes en los topics principales.  
Usé el comando ros2 topic type /scan_raw para obtener el tipo de mensajes en los tópicos principales, una vez metido el comando me salía: sensor_msgs/msg/LaserScan.
Sin embargo, como quería obtener algo más de información sobre el tipo use también el comando ros2 topic info /scan_raw, y obtuve la siguiente información:  
Type: sensor_msgs/msg/LaserScan  
Publisher count: 7  
Subscription count: 6  

4.Observar mensajes en tiempo real en al menos dos topics (uno sensorial y uno de estado).  
Usando el comando ros2 topic echo /scan_raw, obtuve los siguientes resultados: 
header:  
  stamp:  
    sec: 457  
    nanosec: 200000000  
  frame_id: laser_link  
angle_min: -1.5707999467849731  
angle_max: 1.5707999467849731  
angle_increment: 0.008750975131988525  
time_increment: 0.0  
scan_time: 0.0  
range_min: 0.17000000178813934  
range_max: 3.5  
ranges:  
- .inf
En este apartado aparte del inf de repente me salen valores como: 3.4612252712249756, 3.4993715286254883, 3.3562302589416504... y sigue hasta llegar al valor 2.256430149078369.
intensities:  
- 0.0  
- '...'  
---  
header:  
  stamp:  
    sec: 457  
    nanosec: 400000000  
  frame_id: laser_link  
angle_min: -1.5707999467849731  
angle_max: 1.5707999467849731  
angle_increment: 0.008750975131988525  
time_increment: 0.0  
scan_time: 0.0  
range_min: 0.17000000178813934  
range_max: 3.5  
ranges:  
- .inf
Y vuelve a obtener los valores referenciados anteriormente.  

5.Medir la frecuencia de publicación de un topic (por ejemplo, odometría o láser).  
<img width="563" height="221" alt="imagen" src="https://github.com/user-attachments/assets/e79fb72e-dcfa-4b73-8b0c-cdf4771e5e94" />  
En más detalle:  
<img width="732" height="215" alt="imagen" src="https://github.com/user-attachments/assets/2a11c01d-99a3-4f99-a27d-94c2352b7f70" />  


6.Localizar el topic de comandos de velocidad y publicar un comando de prueba (si procede según el sistema).  
<img width="563" height="82" alt="imagen" src="https://github.com/user-attachments/assets/578d3d37-b55f-437d-aca3-4d74bbd10cba" />  

7.Inspeccionar el estado del sistema y visualizar los sensores del robot con RViz2.  
<img width="6924" height="1604" alt="image" src="https://github.com/user-attachments/assets/64ccd403-cd55-4170-8dd1-f9eaa240bbde" />    


