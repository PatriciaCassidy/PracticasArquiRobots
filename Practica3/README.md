**PRÁCTICA 3: Seguimiento de un objeto/persona con detecciones 2D/3D y evitación con láser**  

**Guión de desarrollo incremental**  

**Paso 1: obstáculo más cercano con láser**  
El alumno implementará un nodo que:
	-Reciba un sensor_msgs/msg/LaserScan.
	-Localice el obstáculo más cercano (mínimo rango válido) y calcule
	su posición 2D en el marco del robot.
	-Publique el resultado en /nearest_obstacle como geometry_
	msgs/msg/PointStamped.
	-Publique además una TF propia asociada al obstáculo más cercano.  
	

Requisitos:  
	-El PointStamped publicado en /nearest_obstacle debe estar  
	expresado en el marco del robot (header.frame_id igual al marco
	elegido, p.ej. base_link).  
	-El header.stamp debe ser coherente con el LaserScan de entrada.  
	-La TF del obstáculo más cercano debe publicarse con el mismo
	instante de tiempo (TransformStamped.header.stamp coherente
	con el LaserScan) y como marco hĳo fijo (por ejemplo, nearest_-
	obstacle) respecto al marco del robot (por ejemplo, base_link).  
	-Deben ignorarse rangos inválidos (NaN, Inf o fuera de range_-
	min/range_max).  
	-Si no existe ningún rango válido en el mensaje, no se publicará
	nada.  
	-Si no se puede resolver la transformación TF necesaria en ese
	instante (por ejemplo, porque no está en el buffer), no se publicará
	nada.  	
  
  
Implemento un nodo ROS2 que procesa los datos del láser (LaserScan) que **localize el obstáculo más cercano** y **publique su posición**
 2D en el marco del robot mediante TF.
  
Nodo:obstacle_detector_node
	Suscripciones y publicaciones:  
	-**Entrada**: sensor_msgs/msg/LaserScan  
	-**Salida (posición)**: /nearest_obstacle  →  geometry_msgs/msg/PointStamped  
	-**Salida (TF)**: TF propia asociada al obstáculo más cercano  
  
**Implementación en C++**  
El nodo se **suscribe** al LaserScan con el sensor QoS, luego filtra rangos inválidos (NaN, Inf, fuera de rango (range_min, range_max)) y localiza el minimo; con todo eso ya calcula las coordenadas en el plano del sensor:  
  
  
//
	// Suscripción al láser (QoS sensor)  
	laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(  
  		"input_scan", rclcpp::SensorDataQoS().reliable(),
  		std::bind(&ObstacleDetectorNode::laser_callback, this, _1));  
  
  	// Calcular índice y posición del obstáculo más cercano  
	float min_range = std::numeric_limits<float>::infinity();  
	int   min_idx   = -1;  
	for (size_t i = 0; i < scan.ranges.size(); ++i) {  
  		float r = scan.ranges[i];  
  		if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {  
    			if (r < min_range) { min_range = r; min_idx = i; }  
  		}  
	}  
	if (min_idx < 0) return;  // No hay rango válido, no publicar  
  
	float theta = scan.angle_min + min_idx * scan.angle_increment;  
	float obs_x = min_range * std::cos(theta);  
	float obs_y = min_range * std::sin(theta);  
//
  
  
**Transformación al marco del robot (TF)**  
  
Las coordenadas que he obtenido estan en el origen de coordenadas del sensor
 (LaserScan.header.frame_id), para que las use el robot hay que pasarlas al
  suyo (p.ej. base_link) usando el buffer TF con el instante del propio
   mensaje:  
   
   
//
// Calcular índice y posición del obstáculo más cercano  
float min_range = std::numeric_limits<float>::infinity();  
int   min_idx   = -1;  
for (size_t i = 0; i < scan.ranges.size(); ++i) {  
  float r = scan.ranges[i];  
  if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {  
    if (r < min_range) { min_range = r; min_idx = i; }  
  }  
}  
if (min_idx < 0) return;  // No hay rango válido, no publicar  
  
float theta = scan.angle_min + min_idx * scan.angle_increment;  
float obs_x = min_range * std::cos(theta);  
float obs_y = min_range * std::sin(theta);  
//  
  
**Publico la TF propia del obstaculo**
  
  
//  
geometry_msgs::msg::TransformStamped tf_msg;  
tf_msg.header.stamp = scan.header.stamp;  
tf_msg.header.frame_id = "base_link";  
tf_msg.child_frame_id  = "nearest_obstacle";  
tf_msg.transform.translation.x = obstacle_point_base.point.x;  
tf_msg.transform.translation.y = obstacle_point_base.point.y;  
tf_msg.transform.translation.z = 0.0;  
tf_msg.transform.rotation.w = 1.0;  
tf_broadcaster_->sendTransform(tf_msg);  
//  
  
  
**Requisitos de la entrega**  
  
	-El PointStamped publicado en /nearest_obstacle debe tener 
	header.frame_id = "base_link" (o el marco del robot 
	elegido).  
	-El header.stamp debe ser coherente con el LaserScan de 
	entrada.  
	-La TF publicada debe tener el mismo instante de tiempo que
	el LaserScan y usar "nearest_obstacle" como marco hijo.  
	-Si no existe ningún rango válido, no se publica nada.  
	-Si la transformación TF necesaria no está disponible, no 
	se publica nada.  
  
  
**Validación**  
  
  
//  
# Ver el topic  
ros2 topic echo /nearest_obstacle  
  
# Ver la TF en RViz2 → añadir display TF  
# o consultar desde terminal:  
ros2 run tf2_ros tf2_echo base_link nearest_obstacle  
//  

//
  
  
**Paso 2: detección 2D de un objeto/persona**  
	
El alumno implementará un nodo de detección 2D que:  
	-Reciba una imagen sensor_msgs/msg/Image (topic a elegir/rema-
	pear según el robot o simulador).  
	-Publique una detección como
	vision_msgs/msg/Detection2D (por ejemplo, en /detection_2d).  
	
El alumno puede escoger el método:  
	-Segmentación por color en HSV (solución ligera y explicable).  
	-Deep Learning (por ejemplo, mediante un wrapper tipo Yolo_ROS,
	si está disponible en el entorno).  

Regla importante: si no se detecta nada, no se publica nada.  

  
  
**Paso 3: detección 3D**  

A partir de:  
	-Una vision_msgs/msg/Detection2D.  
	-Una imagen de profundidad (sensor_msgs/msg/Image).  
	-Los parámetros intrínsecos de cámara (sensor_msgs/msg/CameraInfo).  

El alumno implementará un nodo que publique:  
  	-Una vision_msgs/msg/Detection3D (por ejemplo, en /detection_3d).  
  	-Una TF propia asociada al objeto/persona detectado.  
  
Si en un instante dado no hay Detection2D (porque el detector 2D no ha
publicado), entonces este nodo tampoco publicará Detection3D.  

**Pista**: un enfoque mínimo consiste en tomar el centro de la caja 2D, leer su
profundidad 𝑍 y proyectar a 3D usando los intrínsecos de CameraInfo. La TF 
puede publicarse como un marco hĳo (p.ej. target) respecto
al marco óptico de la cámara o respecto a base_link, siempre que se
indique claramente qué marcos se usan.  
  
  
**Paso 4: control de orientación hacia la detección**  
  
El alumno implementará un nodo de control que:  
	-Reciba una vision_msgs/msg/Detection3D.  
	-Genere la velocidad angular necesaria para orientar el robot hacia
	el objetivo.  
	-Publique comandos de velocidad (geometry_msgs/msg/Twist) en
	/cmd_vel.  
  
Si no hay detección, el robot debe girar hacia un lado hasta encontrarla.  
  
Se recomienda implementar el control angular como un PID sobre el
error de orientación (por ejemplo, el ángulo hacia el objetivo en el marco
del robot).  
  
  
**Paso 5: control de distancia (1-2m)**
  
Se ampliará el nodo anterior para que el robot:  
	-Se acerque al objeto/persona hasta una distancia objetivo (entre 1 y
	2 metros).  
	-Si el objeto/persona se acerca demasiado, el robot retroceda para
	mantener la distancia.  
  
  
**Paso 6: evitación con el obstáculo más cercano**  
  
El robot deberá tener en cuenta /nearest_obstacle para evitar colisiones
**cuando el obstáculo interfiera en el seguimiento**. En particular:  

	-La evitación puede inhibir el avance, modificar la velocidad
	angular o imponer una maniobra reactiva.  
	-Debe demostrarse que el robot evita colisiones incluso cuando el
	objetivo visual está “detrás” de un obstáculo.  
