# PRACTICA 4: Navegación con Nav2 y patrullaje mediante FSM

## Descripción

Creamos un sistema de patrullaje autónomo implementado en ROS2 que visita secuencialmente una lista de waypoints usando Nav2 como capacidad de navegación y una FSM como lógica de misión.

La arquitectura separa claramente:
- **Capacidad**: Nav2 — "Ir a una pose objetivo en el mapa"
- **Misión**: FSM de patrullaje — "Visitar repetidamente una lista de poses"

---

## Estructura del paquete

<img width="769" height="434" alt="image" src="https://github.com/user-attachments/assets/1d55d513-f1e7-41e1-9e3a-39a0e578f06f" />


---

## Paso 1: Preparación del entorno y del robot

### 1.1 Lanzar el entorno de simulación

Abre tres terminales. En la primera, lanza la simulación con Gazebo:

```bash
cd ~/ArquiRobots/ros2_ws
source install/setup.bash
ros2 launch kobuki simulation.launch.py
```

En la segunda, lanza Nav2 con el mapa:

```bash
cd ~/ArquiRobots/ros2_ws
source install/setup.bash
ros2 launch kobuki navigation.launch.py
```

O en una sola terminal con el launch combinado:

```bash
ros2 launch kobuki navigation_sim.launch.py
```

### 1.2 Verificar topics relevantes

```bash
ros2 topic list
```

Topics principales del sistema:
- `/scan` — sensor láser
- `/odom` — odometría
- `/cmd_vel` — comandos de velocidad
- `/patrol_state` — estado actual de la FSM (publicado por este paquete)

### 1.3 Verificar transformadas

```bash
ros2 run tf2_tools view_frames
```

Marcos esperados: `odom`, `base_link`, `base_scan` (o equivalentes según el robot Kobuki).

### 1.4 Ver acciones disponibles cuando Nav2 está activo

```bash
ros2 action list
ros2 action info /navigate_to_pose
ros2 interface show nav2_msgs/action/NavigateToPose
```

---

## Paso 2: Capacidad de navegación con Nav2

### 2.1 Verificar que Nav2 está disponible

```bash
ros2 action list | grep navigate_to_pose
```

Debe aparecer `/navigate_to_pose`.

### 2.2 Establecer la pose inicial del robot

En RViz2, usar la herramienta **"2D Pose Estimate"** para indicar la posición y orientación aproximada del robot en el mapa. Las partículas del localizador (AMCL) deben converger hacia la pose correcta.

### 2.3 Validar navegación manual

En RViz2, usar la herramienta **"Nav2 Goal"** para enviar objetivos de navegación manualmente y verificar que el robot navega correctamente y evita obstáculos.

### 2.4 Interfaz NavigationClient utilizada

El paquete `nav2_example` proporciona la clase `NavigationClient` que encapsula la comunicación con Nav2. En `patrol_fsm_node.cpp` se instancia así:

```cpp
nav_client_ = std::make_shared<NavigationClient>();
```

Métodos utilizados en este paquete:

| Método | Uso |
|--------|-----|
| `wait_for_action_server(timeout)` | Verifica si Nav2 está listo (estado INIT) |
| `send_goal(pose)` | Envía un waypoint a Nav2 (estado NAVIGATING) |
| `is_goal_done()` | Comprueba si la navegación terminó |
| `was_goal_successful()` | Comprueba si fue exitosa |
| `cancel_goal()` | Cancela navegación ante timeout |
| `create_pose_stamped(x, y, yaw)` | Construye objetivos de navegación |

---

## Paso 3: Misión de patrullaje con FSM

### 3.1 Diseño de la FSM

La FSM tiene tres estados:

```
INIT ──(Nav2 disponible)──► NAVIGATING ──(éxito)──► NAVIGATING (siguiente WP)
                                │                         │
                            (fallo/timeout)           (ciclo completo)
                                │                         │
                                ▼                         ▼
                           RECOVERY ──(espera 3s)──► NAVIGATING (reintento)
                                │
                           (max reintentos)
                                │
                                ▼
                           NAVIGATING (siguiente WP, saltando fallido)
```

**Estado INIT** (`InitState`):
- Espera a que el servidor de Nav2 esté disponible
- Transición a NAVIGATING cuando Nav2 responde

**Estado NAVIGATING** (`NavigatingState`):
- Envía el waypoint actual a Nav2 al entrar
- Monitoriza progreso y timeout (120s por defecto)
- Transición a NAVIGATING (siguiente WP) si éxito
- Transición a RECOVERY si fallo o timeout

**Estado RECOVERY** (`RecoveryState`):
- Espera 3 segundos antes de reintentar
- Incrementa contador de reintentos
- Transición a NAVIGATING para reintentar (si reintentos < max)
- Salta al siguiente waypoint si se superan los reintentos máximos

### 3.2 Parámetros configurables

En `config/patrol_params.yaml`:

```yaml
patrol_fsm_node:
  ros__parameters:
    max_retries: 3       # Intentos antes de saltar un waypoint
    goal_timeout: 120.0  # Segundos máximos por waypoint
```

### 3.3 Waypoints de patrullaje

Los waypoints están definidos en `patrol_fsm_node.cpp` en el método `load_waypoints()`:

```cpp
waypoints_.push_back(nav_client_->create_pose_stamped(2.0,  2.0,  0.0));
waypoints_.push_back(nav_client_->create_pose_stamped(4.0,  3.0,  1.57));
waypoints_.push_back(nav_client_->create_pose_stamped(6.0,  1.0,  3.14));
waypoints_.push_back(nav_client_->create_pose_stamped(3.0, -1.0, -1.57));
waypoints_.push_back(nav_client_->create_pose_stamped(0.0,  0.0,  0.0));
```

Los argumentos son `(x, y, yaw)` en metros y radianes respecto al marco del mapa.

### 3.4 Observabilidad del sistema

El nodo publica el estado actual en el topic `/patrol_state` con formato:

```
NAVIGATING | WP:2/5 | Retry:0/3
```

Para monitorizar en tiempo real:

```bash
ros2 topic echo /patrol_state
```

Los logs de transiciones se emiten por pantalla con niveles INFO y WARN:

```
[INIT] Esperando servidor Nav2...
[INIT] Nav2 detectado!
[NAVIGATING] Enviando WP 1/5
[NAVIGATING] EXITO!
[NAVIGATING] Enviando WP 2/5
...
CICLO COMPLETO! Reiniciando.
```

### 3.5 Política de recuperación ante fallos

Cuando un objetivo falla o supera el timeout:
1. Se entra en `RecoveryState` y se incrementa el contador de reintentos
2. Se espera 3 segundos
3. Si `retry_count < max_retries` → se reintenta el mismo waypoint
4. Si `retry_count >= max_retries` → se salta al siguiente waypoint y se resetea el contador

---

## Lanzar el sistema completo

**Terminal 1 — Simulación:**
```bash
cd ~/ArquiRobots/ros2_ws && source install/setup.bash
ros2 launch kobuki simulation.launch.py
```

**Terminal 2 — Nav2:**
```bash
cd ~/ArquiRobots/ros2_ws && source install/setup.bash
ros2 launch kobuki navigation.launch.py
```

**Terminal 3 — FSM de patrullaje:**
```bash
cd ~/ArquiRobots/ros2_ws && source install/setup.bash
ros2 launch patrol_fsm patrol_simulation.launch.py
```

**Terminal 4 — Monitorizar estado (opcional):**
```bash
ros2 topic echo /patrol_state
```

---

## Compilación

```bash
cd ~/ArquiRobots/ros2_ws
colcon build --packages-select nav2_example
source install/setup.bash
colcon build --packages-select patrol_fsm --symlink-install
source install/setup.bash
```

---

## Dependencias

- `rclcpp`
- `rclcpp_action`
- `nav2_msgs`
- `nav2_example` (proporciona `NavigationClient`)
- `geometry_msgs`
- `std_msgs`
- `tf2`
