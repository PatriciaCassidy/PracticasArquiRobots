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
  
  
**Transformación al marco del robot (TF)**  
  
Las coordenadas que he obtenido estan en el origen de coordenadas del sensor
 (LaserScan.header.frame_id), para que las use el robot hay que pasarlas al
  suyo (p.ej. base_link) usando el buffer TF con el instante del propio
   mensaje:  
   
   

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
 
  
**Publico la TF propia del obstaculo**
  

	geometry_msgs::msg::TransformStamped tf_msg;  
	tf_msg.header.stamp = scan.header.stamp;  
	tf_msg.header.frame_id = "base_link";  
	tf_msg.child_frame_id  = "nearest_obstacle";  
	tf_msg.transform.translation.x = obstacle_point_base.point.x;  
	tf_msg.transform.translation.y = obstacle_point_base.point.y;  
	tf_msg.transform.translation.z = 0.0;  
	tf_msg.transform.rotation.w = 1.0;  
	tf_broadcaster_->sendTransform(tf_msg);    
  
  
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

	# Ver el topic  
	ros2 topic echo /nearest_obstacle  
	  
	# Ver la TF en RViz2 → añadir display TF  
	# o consultar desde terminal:  
	ros2 run tf2_ros tf2_echo base_link nearest_obstacle  

  
  
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

  

  
**Descripción**  
Implemento un nodo que reciba imágenes de la cámara, detecte el
 objeto de interes y publique la detección 2D (bounding box +
 centro) como vision_msgs/msg/Detection2DArray.  
  
**Métodos de detección disponibles**  
	-**Segmentación por color en HSV** (recomendado).  
	-**Deep Learning** (p.ej. wrapper tipo Yolo_ROS si está
	 disponible en el entorno).  
  
**Nodo: hsv_detector_node (método HSV)**  
  
**Suscripciones y publicaciones**  
	-**Entrada**: sensor_msgs/msg/Image  (topic configurable
	via remapping).  
	-**Salida**: /detection_2d  →  vision_msgs/msg/Detection2DArray  
  
**Pipeline de procesamiento**  
**1.** Convertir Imagen → cv::Mat con cv_bridge (BGR8).  
**2.** Convertir BGR → HSV con cvtColor.  
**3.** Aplicar umbral de color con inRange (parámetros H/S/V min y 
max desde YAML).  
**4.** Calcular momentos y bounding rect de la máscara.  
**5.** Publicar Detection2DArray solo si se detecta algo.  

	// 1. Image -> cv::Mat  
	cv_bridge::CvImagePtr cv_ptr;  
	try {  
		cv_ptr = cv_bridge::toCvCopy(image,  
		sensor_msgs::image_encodings::BGR8);  
	} catch (cv_bridge::Exception & e) { return; }  
	cv::Mat & img_bgr = cv_ptr->image;  
  
	// 2. BGR -> HSV  
	cv::Mat img_hsv, mask;  
	cv::cvtColor(img_bgr, img_hsv, cv::COLOR_BGR2HSV);  
  
	// 3. Umbral HSV (parámetros desde YAML)  
	cv::inRange(img_hsv,  
		cv::Scalar(h_min_, s_min_, v_min_),  
		cv::Scalar(h_max_, s_max_, v_max_), mask);  
  
	// 4. Bounding rect  
	auto moments = cv::moments(mask, true);  
	if (moments.m00 < 100) return;  // Nada detectado, no
	publicar  
	auto bbx = cv::boundingRect(mask);  
  
	// 5. Publicar Detection2DArray  
	vision_msgs::msg::Detection2D det_msg;  
	det_msg.header.frame_id = image->header.frame_id;  
	det_msg.header.stamp    = image->header.stamp;  
	det_msg.bbox.center.position.x = bbx.x + bbx.width  /
	2.0;  
	det_msg.bbox.center.position.y = bbx.y + bbx.height /
	2.0;  
	det_msg.bbox.size_x = bbx.width;  
	det_msg.bbox.size_y = bbx.height;  
  
	vision_msgs::msg::Detection2DArray arr_msg;  
	arr_msg.header = det_msg.header;  
	arr_msg.detections.push_back(det_msg);  
	detection_pub_->publish(arr_msg);  
  
**Parámetros YAML (config/params.yaml)**  

	hsv_detector_node:  
		ros__parameters:  
    			use_sim_time: true  
    			h_min: 35  
    			h_max: 85  
    			s_min: 50  
    			s_max: 255  
    			v_min: 50  
    			v_max: 255  
  
**Regla importante**: si no se detecta nada, no se publica nada.
El nodo aguas abajo (detección 3D) también debe respetar esta
regla.  
  
**Validación**  

	ros2 topic echo /detection_2d  
	# Verificar que bbox.center y size son coherentes con la 
	imagen  
	# Visualizar la máscara en RViz2 (publicar imagen filtrada 
	en un topic debug)  
  
  
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
  

  
**Descripción**  
A parti de la detección 2D, más la imagen de profundidad y los 
parametros de la cámara (CameraInfo), obtengo la posición 3D del 
objeto detectado y la publicare como vision_msgs/msg/Detection3DArray.  
  
**Entradas y salidas**  
	-**Entradas**: : vision_msgs/Detection2DArray  +  
	sensor_msgs/Image (profundidad)  +  sensor_msgs/CameraInfo  
	-**Salida**: /detection_3d  →  vision_msgs/msg/Detection3DArray.  
	-**TF**: TF propia asociada al objeto/persona detectado (p.ej. "target")  
  
**Sincronización de mensajes**  
La detección 2D y la imagen de profundidad llegan por topics
distintos y con timestamps que no coinciden 
exactamente. Uso message_filters con ApproximateTime:  
  

	depth_sub_ =  	std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(  
	  this, "input_depth",  	rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());  
	detection_sub_ =  	std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(  
	  this, "input_detection_2d",   	rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());  
	
	sync_ = 	std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(  
	  MySyncPolicy(10), *depth_sub_, *detection_sub_);  
	sync_->registerCallback(std::bind(&DetectionTo3DNode::callback_sync,  
	  this, _1, _2));  
  
**Proyección pinhole: (u,v,Z) → (X,Y,Z)**  
Se utiliza image_geometry::PinholeCameraModel para evitar manejar manualmente los intrínsecos:  

	// Construir modelo a partir de CameraInfo
	auto model = std::make_shared<image_geometry::PinholeCameraModel>();
	model->fromCameraInfo(info);
  
	// Coordenadas del centro de la bbox 2D  
	int u = static_cast<int>(det2d.bbox.center.position.x);  
	int v = static_cast<int>(det2d.bbox.center.position.y);  
  
	// Leer profundidad Z (distinguir encoding 16UC1 vs 32FC1)  
	float depth = 0.0f;  
	if (depth_msg->encoding == "16UC1") {  
		depth = depth_img.at<uint16_t>(cv::Point2d(u,v)) / 1000.0f; // mm -> m  
	} else {  
		depth = depth_img.at<float>(cv::Point2d(u,v));  
	}  
	if (!std::isfinite(depth) || depth <= 0) return;  // Descartar  
  
	// Proyectar a 3D  
	cv::Point3d ray = model->projectPixelTo3dRay(model-	>rectifyPoint(cv::Point2d(u,v)));  
	ray = ray / ray.z;  // Normalizar para que Z = 1  
	cv::Point3d point = ray * depth;  // Escalar por profundidad  
  
**Publicar Detection3DArray y TF**  

	vision_msgs::msg::Detection3D det3d_msg;  
	det3d_msg.header = depth_msg->header;  
	det3d_msg.bbox.center.position.x = point.x;  
	det3d_msg.bbox.center.position.y = point.y;  
	det3d_msg.bbox.center.position.z = point.z;  
  
	vision_msgs::msg::Detection3DArray arr3d_msg;  
	arr3d_msg.header = det3d_msg.header;  
	arr3d_msg.detections.push_back(det3d_msg);  
	detection_3d_pub_->publish(arr3d_msg);  
  
	// Publicar TF del objeto detectado  
	geometry_msgs::msg::TransformStamped tf_target;  
	tf_target.header = det3d_msg.header;  
	tf_target.child_frame_id = "target";  
	tf_target.transform.translation.x = point.x;  
	tf_target.transform.translation.y = point.y;  
	tf_target.transform.translation.z = point.z;  
	tf_target.transform.rotation.w = 1.0;  
	tf_broadcaster_->sendTransform(tf_target);  
  
**Regla**: si en un instante dado no hay Detection2D (el detector 
2D no ha publicado), este nodo tampoco publicará Detection3D.  
  
**Validación**  

	ros2 topic echo /detection_3d
	# Visualizar en RViz2 → añadir Marker o Pose desde /detection_3d
	ros2 run tf2_ros tf2_echo base_link target
  
  
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
  

  
**Descripción**  
Implemento un nodo de control que reciba la detección 3D y genere
la velocidad angular necesaria para orientar el robot hacia el 
objetivo. Publica comandos de velocidad en /cmd_vel.  

**Nodo: orientation_controller_node**  
**Suscripciones y publicaciones**  
	-**Entrada**: vision_msgs/msg/Detection3DArray  (/detection_3d).  
	-**Salida**: /cmd_vel  →  geometry_msgs/msg/Twist.  
  
**Control PID angular**  
Se implementa un PID sobre el ángulo de error. El ángulo hacia el 
objetivo en el marco del robot se calcula como atan2(y, x) a 
partir 
de la posición 3D de la detección:  

	// Obtener posición del objetivo en el frame del robot  
	// (la Detection3D ya viene en el frame de la cámara;  
	//  transformar a base_link si es necesario)  
	double target_x = detection.bbox.center.position.x;  
	double target_y = detection.bbox.center.position.y;  
  
	// Ángulo de error  
	double angle_error = std::atan2(target_y, target_x);  
  
	// PID angular (simplificado: solo término P)  
	double kp_angular = 1.2;  // Parámetro ajustable  
	double omega = kp_angular * angle_error;  
  
	// Publicar velocidad  
	geometry_msgs::msg::Twist cmd;  
	cmd.angular.z = omega;  
	cmd_vel_pub_->publish(cmd);  
  
**Comportamiento sin detección**  
Si no hay detección, el robot debe girar hacia un lado buscando el 
objetivo:  

	if (detection_3d_array.detections.empty()) {  
		// Girar lentamente buscando el objetivo  
		geometry_msgs::msg::Twist cmd;  
		cmd.angular.z = search_angular_speed_;  // p.ej. 0.3 rad/s  
		cmd_vel_pub_->publish(cmd);  
		return;  
	}  
  
**Parámetros YAML**  

	orientation_controller_node:  
	  ros__parameters:  
	    use_sim_time: true  
	    kp_angular: 1.2  
	    ki_angular: 0.0  
	    kd_angular: 0.1  
	    search_angular_speed: 0.3  
	    max_angular_speed: 1.0  
  
**Validación**  

	ros2 topic echo /cmd_vel  
	# El robot debe orientarse hacia el objeto en RViz2 / simulador  
	# Verificar que cmd.angular.z ≈ 0 cuando el objeto está centrado  
  
  
**Paso 5: control de distancia (1-2m)**  
  
Se ampliará el nodo anterior para que el robot:  
	-Se acerque al objeto/persona hasta una distancia objetivo (entre 1 y
	2 metros).  
	-Si el objeto/persona se acerca demasiado, el robot retroceda para
	mantener la distancia.  
  
  
**Descripción**  
Ampliar el nodo de control anterior para que, además de 
orientarse, 
el robot se acerque o aleje del objetivo para mantenerlo a una 
distancia de entre 1 y 2 metros.  
  
**Control PID de velocidad lineal**  
La distancia al objetivo se obtiene directamente de la coordenada Z 
(profundidad) de la detección 3D. El error de distancia se usa para 
generar la velocidad lineal:  

	double target_distance = detection.bbox.center.position.z;  
	double distance_goal   = 1.5;  // Distancia objetivo en metros  
	double distance_error  = target_distance - distance_goal;  
  
	// PID lineal  
	double kp_linear = 0.5;  
	double vx = kp_linear * distance_error;  
  
	// Limitar velocidad lineal  
	vx = std::clamp(vx, -max_linear_speed_, max_linear_speed_);  

	// Publicar Twist combinado  
	geometry_msgs::msg::Twist cmd;  
	cmd.linear.x  = vx;  
	cmd.angular.z = omega;  // Del paso 4  
	cmd_vel_pub_->publish(cmd);  
  
**Comportamiento ante acercamiento excesivo**  
	-Si el objeto se acerca demasiado (distance_error < 0), el 
	robot retrocede (vx < 0).  
	-La distancia objetivo se configura via parámetro 
	(distance_goal, p.ej. 1.5 m).  
	-Los límites de velocidad se definen como parámetros 
	YAML.  
  
**Parámetros YAML adicionales**  

	orientation_controller_node:  
	  ros__parameters:  
	    # ... parámetros del paso 4 ...  
	    kp_linear: 0.5  
	    ki_linear: 0.0  
	    kd_linear: 0.05  
	    distance_goal: 1.5  
	    max_linear_speed: 0.4  
  
**Validación**  

	# El robot debe mantenerse a ~1.5 m del objeto  
	# Avanzar si el objeto está lejos, retroceder si está 
	demasiado cerca  
	ros2 topic echo /cmd_vel  
	# Verificar que cmd.linear.x ≈ 0 cuando la distancia es 
	~1.5 m  
  
  
**Paso 6: evitación con el obstáculo más cercano**  
  
El robot deberá tener en cuenta /nearest_obstacle para evitar colisiones
**cuando el obstáculo interfiera en el seguimiento**. En particular:  
	-La evitación puede inhibir el avance, modificar la velocidad
	angular o imponer una maniobra reactiva.  
	-Debe demostrarse que el robot evita colisiones incluso cuando el
	objetivo visual está “detrás” de un obstáculo.  
  
  
**Descripción**  
El robot debe combinar el seguimiento del objetivo y el evitar de 
forma reactiva los obstáculos, usando la información del topic 
/nearest_obstacle publicado en el Paso 1.  

**Integración en el nodo de control**  
El nodo de control se suscribe también a /nearest_obstacle y 
modifica los comandos de velocidad cuando un obstáculo interfiere 
con el seguimiento:  
