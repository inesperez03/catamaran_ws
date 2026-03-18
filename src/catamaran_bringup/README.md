# catamaran_bringup

## Qué es

`catamaran_bringup` es el paquete encargado de arrancar el sistema del catamarán.

Su función es:

* generar la descripción del robot desde el Xacro,
* lanzar `ros2_control_node`,
* cargar el hardware interface,
* cargar los controladores necesarios.

---

## Modos de funcionamiento

Actualmente el sistema puede arrancar en dos modos:

* `real`
* `sim`

Por defecto arranca en:

* `real`

Ejemplo:

```bash
ros2 launch catamaran_bringup bringup.launch.py environment:=sim
```

---

## Qué lanza actualmente

Actualmente este paquete lanza:

* `ros2_control_node`
* el hardware interface `CatamaranSystem`
* el controlador de prueba `thruster_test_controller`

---

## Controlador de prueba

El controlador de prueba actual es:

* `thruster_test_controller`

Este controlador permite enviar fuerza directamente a cada thruster por separado a través de:

```text
/thruster_test_controller/commands
```

Tipo de mensaje:

```text
std_msgs/msg/Float64MultiArray
```

Formato:

```text
[left_force, right_force]
```

Ejemplo:

```bash
ros2 topic pub /thruster_test_controller/commands std_msgs/msg/Float64MultiArray "{data: [-20.0, -10.0]}"
```

---

## Para qué sirve ahora

En el estado actual del proyecto, `catamaran_bringup` se usa para:

* arrancar el hardware interface,
* validar el funcionamiento de `ros2_control`,
* probar la conversión de fuerza a PWM en el robot real,
* probar la conversión de fuerza a valores Stonefish en simulación.

---

## Ficheros importantes

* `launch/bringup.launch.py`
  Launch principal del sistema.

* `config/ros2_control_params.yaml`
  Configuración de `controller_manager` y de los controladores.

---

## Estado actual

Ahora mismo este paquete está orientado a pruebas de hardware y validación del sistema base.

Más adelante se ampliará para incluir otros controladores y modos de operación del catamarán.
