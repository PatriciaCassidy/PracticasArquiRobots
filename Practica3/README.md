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
