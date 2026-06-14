# r6bot_description

Paquete ROS 2 **limpio y autocontenido** con la descripcion URDF del robot
**r6bot**, generado a partir de las mallas STL del `lab3`
(`base_link`, `link_1`, `link_2`, `link_3`, `link_4`, `link_5`).

Contiene **solo lo necesario** para leer y visualizar el URDF.

## Estructura

```
r6bot_description/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── display.launch.py        # robot_state_publisher + sliders + RViz
├── meshes/                       # mallas STL (base_link, link_1..link_5)
├── rviz/
│   └── r6bot.rviz
└── urdf/
    ├── create_link.xacro         # macro auxiliar (visual+colision desde STL)
    └── r6bot.urdf.xacro          # URDF principal (PLANTILLA, ajustar offsets)
```

## Uso

1. Copia/clona esta carpeta dentro del `src/` de tu workspace, por ejemplo:

   ```bash
   cp -r r6bot_description ~/ros2_ws/src/
   ```

2. Compila e instala:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select r6bot_description
   source install/setup.bash
   ```

3. Visualiza con sliders en RViz:

   ```bash
   ros2 launch r6bot_description display.launch.py
   ```

## Notas importantes

- El `r6bot.urdf.xacro` es una **PLANTILLA**: los `origin` (xyz/rpy) de cada
  `joint` son valores de ejemplo apilados sobre el eje Z. **Ajustalos con las
  mediciones reales** de tu imagen de referencia para que la cinematica
  coincida con el robot real.
- Se usa el **mismo STL** para `visual` y `collision` porque solo hay archivos
  `.stl` (no hay `.dae`).
- Las mallas subidas son 6 (`base_link` + `link_1..link_5`), lo que da
  **5 articulaciones revolutas**. Si falta un eslabon para tener las 6 DOF
  completas del r6bot, agrega la malla correspondiente y un `joint` adicional.
- Las inercias son genericas (placeholder); ajustalas solo si necesitas
  dinamica o simulacion en Gazebo.
