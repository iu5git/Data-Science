# gazebo_sim

Launch файлы, миры Gazebo, конфигурация waypoints и настройки симуляции.

---

## Launch файлы

### `launch.py` — основной запуск

Запускает симуляцию с одним роботом. Параметры:

| Параметр | Значение по умолчанию | Описание |
|----------|----------------------|----------|
| `use_python_controller` | `false` | Python или C++ контроллер |
| `world` | `src/gazebo_sim/worlds/empty.sdf` | Мир Gazebo |
| `rviz_config` | `src/gazebo_sim/config/robot_view.rviz` | Конфиг RViz |

Запуск через Makefile:
```bash
make gazebo-py    # Python контроллер
make gazebo-cpp   # C++ контроллер
```

### `gazebo_multi_nav2_world.launch.py` — мультироботный запуск с Nav2

Поддерживает несколько роботов, каждый со своим namespace и Nav2.

**Настройка роботов** — файл `robot.config` в формате:
```yaml
robots:
  - namespace: robot1
    spawn_pose: [0.0, 0.0, 0.0, 0.0]
  - namespace: robot2
    spawn_pose: [1.0, 1.0, 0.0, 0.0]
```

**Смена модели робота** (строка 102 в launch файле):
- `go2_description` — Unitree Go2 (по умолчанию)
- `go1_description` — Unitree Go1

---

## Миры Gazebo

| Файл | Описание |
|------|----------|
| `worlds/empty.sdf` | Пустой мир, только пол |
| `worlds/simple_world.sdf` | Мир с препятствиями для навигации |

Путь к мирам задаётся параметром `world` в launch файле.

---

## Waypoints

Waypoints хранятся в `config/waypoints/` в формате YAML (список):

```yaml
- # 1
  x: 1.0
  y: 1.0
  z: 0.0
  yaw: 0.0
```

Загрузка:
```bash
make waypoint-load FILE=default
```

---

## RViz конфигурация

- `config/robot_view.rviz` — базовая конфигурация для одного робота
- Отображает: TF, robot model, laser scan, map, Nav2 path, waypoints (через rviz_waypoint_tool)

---

## Зависимости пакета

- gazebo_ros2_control
- ros_gz_sim
- robot_localization (EKF)
- Nav2 стек
- go1_description / go2_description
