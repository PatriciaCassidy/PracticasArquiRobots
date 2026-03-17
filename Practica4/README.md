**PRACTICA 4: Navegación con Nav2 y patrullaje mediante FSM**  

**4.3. Guión de desarrollo**  
La práctica se desarrolla siguiendo una secuencia incremental de pasos,
donde cada paso produce un sistema funcional y verificable antes de
proceder con el siguiente. El alumno debe completar cada paso de forma
ordenada y validar su correcto funcionamiento antes de continuar.  

**Paso 1: Preparación del entorno y del robot**  
En este paso inicial, el alumno debe verificar que el robot está correcta-
mente configurado y que dispone de todos los sensores y transformadas
necesarios para la navegación.  
1. Lanzar el entorno de simulación del robot proporcionado así como
la capacidad de navegación.  
2. Abrir RViz2 y verificar la configuración visual del robot.  
3. Identificar y listar los topics relevantes:  
Topic de sensor láser (normalmente /scan).  
Topic de odometría (normalmente /odom).  
Topic de comandos de velocidad (normalmente /cmd_vel).  
4. Verificar que las transformadas del robot están correctamente
publicadas:  
Ejecutar ros2 run tf2_tools view_frames para generar el
grafo de transformadas.  
Confirmar la presencia de los marcos odom, base_link y
base_scan (o equivalentes según el robot).  
  
Ver topics disponibles cuando Nav2 está en ejecución  
ros2 topic list  

Ver acciones disponibles  
ros2 action list  

Ver tipo de la acción NavigateToPose  
ros2 action info /navigate_to_pose  

Ver estructura del mensaje de la acción  
ros2 interface show nav2_msgs/action/NavigateToPose  

// navigation_client.hpp
class NavigationClient : public rclcpp::Node
{
public:
  // Alias para simplificar
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // Métodos principales
  bool wait_for_action_server(std::chrono::seconds timeout);
  void send_goal(const geometry_msgs::msg::PoseStamped& target_pose);
  bool is_goal_active() const { return goal_active_; }
  bool is_goal_done() const { return goal_done_; }
  bool was_goal_successful() const { return goal_success_; }
  std::shared_ptr<const NavigateToPose::Feedback> get_feedback() const;
  void cancel_goal();
  
  // Callbacks internos
  void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleNav::SharedPtr, 
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult & result);
};  


// simple_navigation_app.cpp - Fragmento clave
void SimpleNavigationApp::control_cycle()
{
  // FASE 1: Esperar disponibilidad del servidor
  if (!server_ready_) {
    if (nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      server_ready_ = true;
    }
    return;
  }
  
  // FASE 2: Enviar objetivo (solo una vez)
  if (!goal_sent_) {
    nav_client_->send_goal(target_pose_);
    goal_sent_ = true;
    return;
  }

  // FASE 3: Monitorizar progreso
  if (!nav_client_->is_goal_done()) {
    auto feedback = nav_client_->get_feedback();
    if (feedback) {
      // Mostrar progreso
    }
    return;
  }
  
  // FASE 4: Procesar resultado
  if (nav_client_->was_goal_successful()) {
    // Éxito
  } else {
    // Fallo
  }
}  

// simple_navigation_app.cpp - Fragmento clave
void SimpleNavigationApp::control_cycle()
{
  // FASE 1: Esperar disponibilidad del servidor
  if (!server_ready_) {
    if (nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      server_ready_ = true;
    }
    return;
  }
  
  // FASE 2: Enviar objetivo (solo una vez)
  if (!goal_sent_) {
    nav_client_->send_goal(target_pose_);
    goal_sent_ = true;
    return;
  }

  // FASE 3: Monitorizar progreso
  if (!nav_client_->is_goal_done()) {
    auto feedback = nav_client_->get_feedback();
    if (feedback) {
      // Mostrar progreso
    }
    return;
  }
  
  // FASE 4: Procesar resultado
  if (nav_client_->was_goal_successful()) {
    // Éxito
  } else {
    // Fallo
  }
}  



**Paso 2: Capacidad de navegación con Nav2**  
En este paso se configura y valida la navegación autónoma mediante
Nav2, que constituirá la capacidad fundamental sobre la que se construirá
la tarea de patrullaje.
1. Lanzar Nav2 con indicando el mapa sobre el cual navegar:  
Ejecutar el launch file de Nav2 proporcionado, especificando
la ruta al archivo de mapa (map.yaml).  
4 Navegación con Nav2 y patrullaje mediante FSM 140
Verificar en RViz2 que el mapa se visualiza correctamente.
Comprobar que la acción de navegación está disponible ejecu-
tando:  
ros2 action list y verificando la presencia de /navigate_-
to_pose.  
2. Establecer la pose inicial del robot:  
Usar la herramienta “2D Pose Estimate” de RViz2.
Indicar la posición y orientación aproximada del robot en el
mapa.  
Observar cómo las partículas convergen hacia la pose correcta.  
3. Validar la navegación mediante objetivos manuales:  
Usar la herramienta “Nav2 Goal” de RViz2 para enviar objeti-
vos de navegación.  
Observar el comportamiento ante obstáculos (evitación diná-
mica).  

cd ~/ros2_ws/src
ros2 pkg create patrol_fsm --build-type ament_cmake --dependencies rclcpp geometry_msgs nav2_msgs rclcpp_action tf2  

**Paso 3: Misión de patrullaje con FSM**  
En este paso final, el alumno implementará una FSM que coordina una
tarea de patrullaje mediante la capacidad de navegación validada en el
paso anterior.  
1. Diseñar la FSM de patrullaje:  
Definir los estados necesarios.
Establecer las transiciones entre estados basadas en eventos
de Nav2.  
Decidir la política de reintentos ante fallos de navegación.  
2. Crear un nuevo paquete ROS 2 para el comportamiento de patru-
llaje:  
Usar ros2 pkg create con las dependencias necesarias (rclcpp,
nav2_msgs, geometry_msgs).  
Estructurar el código siguiendo alguno de los patrones de
FSM estudiados en la sección de teoría.  
3. Implementar un cliente de acción para Nav2:  
Crear un action client del tipo NavigateToPose.
Implementar callbacks para goal response, feedback y result.
Gestionar adecuadamente los estados del action (pending,
active, succeeded, aborted, canceled).  
4. Definir una lista de waypoints para el patrullaje:  
Inicialmente, codificar una lista estática de waypoints en el
código.
Cada waypoint debe incluir posición (x, y) y orientación (theta
o quaternion).
Opcionalmente, cargar los waypoints desde un archivo de
parámetros.
5. Implementar la lógica de la FSM:  
Al activarse, la FSM debe enviar el primer waypoint como
goal a Nav2.
4 Navegación con Nav2 y patrullaje mediante FSM 141
Al recibir confirmación de éxito, avanzar al siguiente way-
point.
Al completar todos los waypoints, reiniciar el ciclo de patru-
llaje.
Ante fallos de navegación, implementar una política de rein-
tentos (por ejemplo, 3 intentos antes de omitir el waypoint).   
6. Añadir observabilidad al comportamiento:  
Publicar el estado actual de la FSM en un topic (std_msgs/String
o mensaje personalizado).
Publicar el waypoint actual y el progreso de la misión.
Implementar logging adecuado de transiciones de estado y
eventos relevantes.  
7. Validar el comportamiento completo:  
Lanzar el sistema completo (simulación + Nav2 + FSM de
patrullaje).
Observar que el robot visita secuencialmente todos los way-
points.
Provocar un fallo de navegación (bloqueando el camino) y
verificar la recuperación.
Confirmar que el patrullaje se ejecuta de forma continua y
autónoma.
Al finalizar este paso, el alumno dispondrá de un sistema completo
de patrullaje autónomo que demuestra la separación clara entre capa-
cidad (Nav2) y tarea (FSM de patrullaje), consolidando los conceptos
arquitectónicos del modelo misión–tarea–capacidad.
