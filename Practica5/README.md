# Práctica 5 – Robot Camarero con Behavior Trees

**Asignatura:** Arquitecturas Software para Robots (ASR)  
**Paquete ROS 2:** `waiter_robot`  
**Simulador:** Gazebo + Nav2 (ROS 2 Jazzy)

---

## Índice

1. [Descripción](#descripción)
2. [Arquitectura del sistema](#arquitectura-del-sistema)
3. [Estructura del paquete](#estructura-del-paquete)
4. [Nodos BT implementados](#nodos-bt-implementados)
5. [Árbol de comportamiento](#árbol-de-comportamiento)
6. [Guión de desarrollo paso a paso](#guión-de-desarrollo-paso-a-paso)
7. [Cómo compilar](#cómo-compilar)
8. [Cómo ejecutar](#cómo-ejecutar)
9. [Errores corregidos](#errores-corregidos)

---

## Descripción

Se implementa un **robot camarero** que utiliza un **Behavior Tree (BT)** para orquestar una misión completa de atención al cliente en un entorno simulado:

1. El robot parte de una posición inicial (*home*).
2. Espera la llegada de un cliente.
3. Toma el pedido (bebida + comida) mediante reconocimiento de voz simulado.
4. Navega a la cocina, recoge el pedido y vuelve con el cliente.
5. Entrega el pedido y se despide.

El ciclo se repite indefinidamente mientras el nodo esté activo.

---

## Arquitectura del sistema

```
┌─────────────────────────────────────────────┐
│              waiter_bt_example              │  ← ejecutable principal
│                                             │
│   BT::BehaviorTreeFactory                   │
│     ├─ NavigateToPose  ──► Nav2 action srv  │
│     ├─ SayText         ──► RCLCPP_INFO      │
│     ├─ ListenText      ──► simulado (STT)   │
│     └─ ExtractInfo     ──► extracción kw    │
│                                             │
│   Blackboard: home_x/y, client_x/y,        │
│               kitchen_x/y, bebida, comida  │
└─────────────────────────────────────────────┘
         │
         ▼ /navigate_to_pose (action)
┌─────────────────┐
│   Nav2 Stack    │  (bt_navigator + planner + controller + costmaps)
└─────────────────┘
         │
         ▼ /cmd_vel
┌─────────────────┐
│  TurtleBot3 /   │
│  robot simulado │
└─────────────────┘
```

---

## Estructura del paquete

```
waiter_robot/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── waiter_tree.xml        ← Behavior Tree principal
│   ├── waiter_params.yaml     ← coordenadas home / cliente / cocina
│   ├── nav2_params.yaml       ← configuración Nav2
│   └── hri_params.yaml        ← parámetros HRI (futuro)
├── include/waiter_robot/
│   ├── bt_nodes/
│   │   └── bt_node_registration.hpp
│   ├── navigate_to_pose_action.hpp
│   ├── say_text_action.hpp
│   ├── listen_text_action.hpp
│   ├── extract_info_action.hpp
│   └── text_utils.hpp
├── src/
│   ├── bt_nodes/
│   │   └── bt_node_registration.cpp
│   ├── navigate_to_pose_action.cpp
│   ├── say_text_action.cpp
│   ├── listen_text_action.cpp
│   ├── extract_info_action.cpp
│   ├── text_utils.cpp
│   └── waiter_bt_example.cpp  ← main
├── launch/
│   └── waiter.launch.py
└── maps/
    └── map.yaml
```

---

## Nodos BT implementados

| Nodo BT | Tipo | Puertos | Descripción |
|---|---|---|---|
| `NavigateToPose` | `StatefulActionNode` | in: `x`, `y`, `yaw` | Envía goal al servidor Nav2 y espera resultado |
| `SayText` | `SyncActionNode` | in: `text` | Imprime el texto (soporta `{variable}`) |
| `ListenText` | `SyncActionNode` | out: `recognized_text` | Simula ASR; devuelve frase fija |
| `ExtractInfo` | `SyncActionNode` | in: `interest`, `full_text`; out: `extracted_info` | Extrae bebida o comida del texto |

Todos los nodos custom se registran en `register_waiter_nodes()` siguiendo el patrón del profesor (lambda con `registerBuilder`).

---

## Árbol de comportamiento

```
WaiterMission  [Sequence]
 ├─ GoHome          [Sequence]
 │    ├─ SayText("Yendo a posicion inicial")
 │    ├─ NavigateToPose(home_x, home_y, home_yaw)
 │    └─ SayText("En posicion de espera")
 │
 ├─ WaitForClient   [Sequence]
 │    ├─ SayText("Esperando cliente...")
 │    ├─ Timeout(3 s) → AlwaysSuccess
 │    └─ SayText("Cliente detectado!")
 │
 ├─ TakeOrder       [Sequence]
 │    ├─ SayText("Hola! Soy su camarero robot")
 │    ├─ GetBeverage [Sequence]
 │    │    ├─ ListenText → {recognized_text}
 │    │    ├─ ExtractInfo(bebida) → {bebida}
 │    │    └─ SayText("Bebida anotada: {bebida}")
 │    ├─ GetFood    [Sequence]
 │    │    ├─ ListenText → {recognized_text}
 │    │    ├─ ExtractInfo(comida) → {comida}
 │    │    └─ SayText("Comida anotada: {comida}")
 │    └─ SayText("Pedido completo: {bebida} y {comida}")
 │
 ├─ GoToKitchen     [Sequence]
 │    ├─ SayText("Yendo a la cocina")
 │    ├─ NavigateToPose(kitchen_x, kitchen_y, kitchen_yaw)
 │    └─ SayText("Llegue a la cocina")
 │
 ├─ PickUpOrder     [Sequence]
 │    ├─ SayText("Recogiendo pedido...")
 │    ├─ Timeout(3 s) → AlwaysSuccess
 │    └─ SayText("Pedido recogido!")
 │
 ├─ ReturnToClient  [Sequence]
 │    ├─ SayText("Volviendo al cliente")
 │    ├─ NavigateToPose(client_x, client_y, client_yaw)
 │    └─ SayText("Aqui tiene su pedido!")
 │
 └─ DeliverOrder    [Sequence]
      ├─ SayText("Entregando pedido...")
      ├─ Timeout(3 s) → AlwaysSuccess
      └─ SayText("Buen provecho!")
```

---

## Guión de desarrollo paso a paso

### Paso 1 – Crear el paquete

```bash
cd ~/ArquiRobots/ros2_ws/PracticasArquiRobots/Practica5/ws_waiter/src
ros2 pkg create waiter_robot --build-type ament_cmake --dependencies rclcpp rclcpp_action nav2_msgs geometry_msgs behaviortree_cpp
```

### Paso 2 – Implementar `NavigateToPoseAction`

Nodo `StatefulActionNode` que actúa como cliente del action server `/navigate_to_pose` de Nav2.  
Patrón: `onStart()` envía el goal, `onRunning()` comprueba si llegó el resultado, `onHalted()` cancela.

### Paso 3 – Implementar `SayText`, `ListenText`, `ExtractInfo`

Nodos `SyncActionNode` sencillos:
- `SayText`: lee el puerto `text`, expande variables del blackboard con `formatText()` y hace `RCLCPP_INFO`.
- `ListenText`: simula reconocimiento de voz devolviendo una frase fija por el puerto `recognized_text`.
- `ExtractInfo`: busca palabras clave en el texto reconocido y escribe la categoría extraída (`bebida`/`comida`) por el puerto `extracted_info`.

### Paso 4 – Implementar `text_utils` (formatText)

Función auxiliar que sustituye `{variable}` por el valor correspondiente del blackboard usando `std::regex`.  
Permite que `SayText` muestre mensajes dinámicos como *"Bebida anotada: café"*.

### Paso 5 – Registrar los nodos en `bt_node_registration`

Función `register_waiter_nodes()` que registra cada clase con `factory.registerBuilder<T>()` usando lambdas que capturan el nodo ROS 2.

### Paso 6 – Diseñar el árbol XML (`waiter_tree.xml`)

Árbol en XML siguiendo el formato BehaviorTree.CPP v4 (`BTCPP_format="4"`).  
Puntos clave:
- `Timeout` usa atributo `msec` (no `duration`).
- Los nombres de atributos en el XML deben coincidir **exactamente** con los nombres de puertos declarados en `providedPorts()`.
- Las variables del blackboard se referencian como `{nombre}`.

### Paso 7 – Implementar `waiter_bt_example.cpp`

Main que:
1. Crea el nodo ROS 2.
2. Registra los nodos BT.
3. Carga el XML del árbol.
4. Inicializa el blackboard con los parámetros de posición.
5. Ejecuta el bucle de ticks a 10 Hz; al terminar cada ciclo llama a `tree.haltTree()` y reinicia.

### Paso 8 – Configurar el launch

`waiter.launch.py` lanza Nav2 (con el mapa y params) y el nodo del camarero pasándole `waiter_params.yaml`.

### Paso 9 – Compilar y ejecutar en simulación

Ver sección [Cómo compilar](#cómo-compilar).

---

## Cómo compilar

```bash
cd ~/ArquiRobots/ros2_ws/PracticasArquiRobots/Practica5/ws_waiter
colcon build --symlink-install --packages-select waiter_robot
source install/setup.bash
```

---

## Cómo ejecutar

### Terminal 1 – Simulador (TurtleBot3 en Gazebo)

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 – Robot camarero + Nav2

```bash
source ~/ArquiRobots/ros2_ws/PracticasArquiRobots/Practica5/ws_waiter/install/setup.bash
ros2 launch waiter_robot waiter.launch.py use_sim_time:=true
```

### Terminal 3 – Visualización (opcional)

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix waiter_robot)/share/waiter_robot/rviz/waiter.rviz
```

### Grabar vídeo (screencast)

```bash
# Instalar si no está disponible:
sudo apt install kazam   # o simplemente usar OBS Studio

# Con ffmpeg (captura pantalla directa):
ffmpeg -video_size 1920x1080 -framerate 25 -f x11grab -i :0.0 \
       -codec:v libx264 demo_waiter.mp4
# Ctrl+C para detener
```

### Parámetros configurables (`config/waiter_params.yaml`)

```yaml
waiter_bt:
  ros__parameters:
    use_sim_time: true
    home_x:      -2.0    # posicion de espera del robot
    home_y:       0.0
    home_yaw:     0.0
    client_x:     2.0    # posicion del cliente
    client_y:    -2.0
    client_yaw:   0.0
    kitchen_x:    2.0    # posicion de la cocina/barra
    kitchen_y:    5.0
    kitchen_yaw:  0.0
```

---

## Errores corregidos

| Archivo | Error | Corrección |
|---|---|---|
| `waiter_tree.xml` | `<Timeout duration="3">` no reconocido en BTCPP v4 | Cambiado a `<Timeout msec="3000">` |
| `waiter_tree.xml` | Falta atributo `BTCPP_format="4"` en `<root>` | Añadido |
| `say_text_action.cpp` | `{bebida}` se imprimía literal sin expandir | Se llama a `formatText()` antes de imprimir |
| `navigate_to_pose_action.cpp` | `goal_response_callback` con firma antigua (`shared_future`) incompatible con Jazzy | Firma actualizada a `const GoalHandleNav::SharedPtr &` |
| `waiter_bt_example.cpp` | Bucle con `status != FAILURE` paraba en fracaso; no se reiniciaba correctamente el árbol | Bucle `while(rclcpp::ok())` + `tree.haltTree()` tras SUCCESS/FAILURE |
| `waiter.launch.py` | `parameters=[LaunchConfiguration('use_sim_time')]` – tipo incorrecto | Cambiado a `parameters=[{'use_sim_time': ...}, yaml_file]` |
| `CMakeLists.txt` | `tf2` no declarado como dependencia de `find_package` ni en `ament_target_dependencies` | Añadido `find_package(tf2 REQUIRED)` y al target |
| `package.xml` | `<depend>tf2</depend>` ausente | Añadido |
