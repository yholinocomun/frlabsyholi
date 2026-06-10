# Laboratorio 6 — Control por Cinemática Diferencial y Cinemática Inversa del UR5

Fundamentos de Robótica · UTEC · 2026-1

Este paquete implementa el control cinemático del robot **UR5** mediante
cinemática diferencial (Jacobiano + pseudo-inversa) y la cinemática inversa de
posición y orientación usando la librería **kinpy**.

---

## 1. Contenido del paquete

```
lab6/
├── lab6functions.py        # Funciones base (FK, Jacobianos, cuaterniones)
├── lab6/
│   ├── test_diffkine.py    # Act 1.1 — Control cinemático de POSICIÓN
│   ├── ik_kinpy.py         # Act 2.1 — IK posición+orientación (RViz)
│   ├── ik_gazebo.py        # Act 2.2 — IK posición+orientación (Gazebo)
│   ├── move_orientation.py # Act 3.1 — Cambio de SOLO orientación (plano ZY)
│   └── test_kinpy          # Ejemplo mínimo de uso de kinpy
├── urdf/ur5_robot.urdf     # Modelo del robot para kinpy
└── launch/                 # Lanzadores de visualización en RViz
```

### Funciones completadas en `lab6functions.py`
- `dh(d, theta, a, alpha)` — matriz de transformación homogénea (Denavit-Hartenberg).
- `fkine_ur5(q)` — cinemática directa del UR5.
- `jacobian_position(q)` — Jacobiano analítico de **posición** (3×6) por diferencias finitas.
- `jacobian_pose(q)` — Jacobiano analítico de **pose** (7×6: posición + cuaternión).
- `rot2quat`, `TF2xyzquat`, `norm_ang` — utilidades de orientación.

---

## 2. Instalación

```bash
# Dependencias de Python
pip install kinpy scipy numpy

# Copiar este paquete al workspace y compilar
cd ~/lab_ws
colcon build --packages-select lab6
source install/setup.bash
```

---

## 3. Cómo ejecutar cada actividad

### Actividad 1.1 — Control cinemático de posición (`test_diffkine`)
Mueve el efector final a una posición deseada usando cinemática diferencial,
la pseudo-inversa de Moore-Penrose y ganancia `k = 1`.

```bash
# Terminal 1: visualización del robot
ros2 launch lab6 view_ur_without_sliders.launch.py ur_type:=ur5

# Terminal 2: control
ros2 run lab6 test_diffkine
```

Ley de control implementada (resumen):
```
x  = fkine_ur5(q)[0:3,3]      # posición actual
e  = x - xd                   # error
e_dot = -k * e                # ley de control
dq = pinv(J) @ e_dot          # velocidad articular
q  = q + dt * dq              # integración de Euler
```

**Pregunta de la guía (ganancias máx/mín):** edita `k` en `test_diffkine.py`
y prueba valores como `0.05, 0.1, 0.5, 1, 5, 10`. Anota:
- **k muy bajo** → el error converge demasiado lento y no alcanza la meta.
- **k muy alto** → el sistema oscila/diverge (inestable).

Los registros se guardan en `~/xcurrent.txt`, `~/xdesired.txt`, `~/q.txt`.

### Actividad 2.1 — IK en visualización (`ik_kinpy`)
Resuelve la IK de posición + orientación con kinpy y muestra el resultado en RViz.

```bash
# Terminal 1
ros2 launch lab6 view_ur_without_sliders.launch.py ur_type:=ur5
# Terminal 2
ros2 run lab6 ik_kinpy
```
Edita `target_pos` y `desired_rpy` dentro de `ik_kinpy.py` para tu pose deseada.

### Actividad 2.2 — IK en simulación (`ik_gazebo`)
Igual que la anterior, pero envía la configuración al controlador de Gazebo.

```bash
# Terminal 1: simulación del UR5 en Gazebo (paquete de UniversalRobots)
# Terminal 2
ros2 run lab6 ik_gazebo
```

### Actividad 3.1 — Cambio de solo orientación (`move_orientation`)
Mantiene la POSICIÓN del efector fija y solo cambia su ORIENTACIÓN, barriendo
un ángulo de modo que el movimiento ocurra en un plano paralelo al plano ZY
(rotación alrededor del eje X). Incluye la restricción de seguridad de no bajar
de 0.10 m sobre la base.

```bash
# Terminal 1: simulación del UR5 en Gazebo
# Terminal 2
ros2 run lab6 move_orientation
```
Ajusta `pos_fija` (la posición debe quedar 5 cm debajo del último link) y el
rango de `angulos` según lo solicitado.

### Actividad 4.1 — Robot real
Es la misma lógica de `move_orientation`, pero conectando el driver del robot
real (ver Sección 4 de la guía: IP estática, URCap *External Control* y
`ros2 launch ur_robot_driver ur5_bringup.launch robot_ip:=<IP>`).
Enviar el código por correo indicando el nombre de **ambos integrantes**.

---

## 4. Notas importantes
- Las poses deseadas (`target_pos`, `desired_rpy`, `pos_fija`) son ejemplos:
  cámbialas según lo que pida el profesor.
- `ee_link` y `base_link` deben coincidir con los nombres definidos en el URDF.
- Para Gazebo/real, el tópico de la acción es
  `/joint_trajectory_controller/follow_joint_trajectory`.
