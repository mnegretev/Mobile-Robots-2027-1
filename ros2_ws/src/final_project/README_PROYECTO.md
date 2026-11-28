# Proyecto final: robot de servicio por lenguaje natural

El robot simulado (base diferencial con brazo xArm6 en la casa de Gazebo) recibe órdenes por voz,
las ejecuta en el simulador y responde con voz. Cuando no puede hacer algo, dice por qué.

```
micrófono → speech2text (Whisper) → /sp_rec/recognized → voice_commander → /tts_query → text2speech (Piper) → bocina
                                                          │
                                                          ├─ /goal_pose → A* + pure pursuit      (navegar)
                                                          ├─ /camera/color/image_raw → color/YOLO (reconocer)
                                                          ├─ /xarm6_traj_controller/...           (brazo)
                                                          └─ Ollama (llama3), opcional            (preguntas generales)
```

## Archivos
| Archivo | Qué hace |
|---|---|
| `final_project/nlu.py` | Entiende la frase en español y la convierte en intenciones (sin ROS, se puede probar solo). |
| `final_project/voice_commander.py` | Nodo ROS 2 que ejecuta las intenciones y habla. |
| `config/knowledge.yaml` | Lugares (x, y, yaw), objetos (lugar, color, clase YOLO) y poses del brazo. **Edítalo para ajustar.** |
| `launch/voice_robot.launch.py` | Lanza navegación + GUI + voz + el nodo del proyecto. |

## Requisitos (una sola vez)
- Lo que ya usan `speech2text` y `text2speech`: `faster-whisper`, `pyaudio`, `piper-tts` y el modelo
  `es_MX-claude-high.onnx` (+ `.json`) en `src/hri/text2speech/models/` (ver `models.txt`).
- Opcional, para reconocer objetos con YOLO: `pip install ultralytics` (descarga `yolov8n.pt` la primera vez).
  Si no está, el robot reconoce objetos por color.
- Opcional, para preguntas generales: [Ollama](https://ollama.com) con `ollama pull llama3`.
  Si no está, el robot contesta "No sé la respuesta" a las preguntas que no conoce.

## Ejecución
```bash
cd ~/Mobile-Robots-2027-1/ros2_ws
colcon build --packages-select final_project && source install/local_setup.bash

# Terminal 1: simulación
ros2 launch house_simul house_simul.launch.py

# Terminal 2: todo lo demás
ros2 launch final_project voice_robot.launch.py
#   opciones: use_mic:=false  use_tts:=false  use_llm:=false  use_yolo:=false  require_name:=true
```

Para probar sin micrófono (en otra terminal):
```bash
ros2 topic pub --once /voice_commander/text std_msgs/msg/String "{data: 've a la cocina'}"
```

## Frases para la demostración
| Habilidad | Ejemplos | Respuesta cuando NO puede |
|---|---|---|
| Navegar | "ve a la cocina", "llévame a la sala", "regresa" | "ve al baño" → "No puedo ir a baño porque no lo conozco…" |
| Reconocer objetos | "¿qué ves?", "busca la mostaza" | "busca la pizza" → "No conozco el objeto pizza…"; si no la ve, lo dice |
| Manipulador | "levanta el brazo", "baja el brazo", "estira el brazo", "saluda", "toma la mostaza", "tráeme la lata" | "mueve el brazo en círculos" → "No conozco ese movimiento…"; si el controlador no está activo, lo dice |
| Preguntas | "¿cómo te llamas?", "¿dónde estás?", "¿dónde está la taza?", "¿qué hora es?", "¿qué puedes hacer?", "¿cuál es la capital de Francia?" (LLM) | "¿dónde está el control?" → "No sé dónde está control…" |
| Imposibles | "vuela", "abre la puerta", "cocina unos huevos" | "Lo siento, no puedo volar. No tengo esa capacidad." |
| Compuestas | "ve a la cocina y luego busca la mostaza" | Si un paso falla, no hace el resto y lo avisa |
| Detener | "alto", "detente" | — |

## Ajustes frecuentes
- **El brazo choca o la cámara no apunta a la mesa:** mueve el brazo con la GUI hasta una buena pose, copia los 6 ángulos a
  `arm_poses` en `knowledge.yaml` (`mirar`, `pre_tomar`, `tomar`, `cargar`) y recompila.
- **El robot no llega a un lugar:** cambia `x`, `y`, `yaw` del lugar. El nodo mueve la meta a la celda libre más cercana del mapa inflado.
- **No reconoce un objeto:** revisa `colors`/`yolo` del objeto. Con `grasp_requires_detection:=false` el robot intenta tomar sin confirmar con la cámara.
- **El robot se escucha a sí mismo:** sube `seconds_per_char` (por defecto 0.075 s por letra).
- **La pinza no responde:** verifica el nombre del controlador con `ros2 topic list | grep gripper` y ajústalo en `gripper.topic`.

Nota: la toma de objetos es una secuencia de poses predefinidas (no se calcula con cinemática inversa),
porque `ik_numeric/nr.py` todavía tiene los TODO sin resolver.
