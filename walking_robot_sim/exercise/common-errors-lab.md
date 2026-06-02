# Типовые ошибки при выполнении лабораторных работ

## ЛР0 — Установка и запуск

### Робот перевернулся на спину с вытянутыми ногами

**Ошибка:** После запуска `make gazebo-cpp` робот лежит на спине, ноги сильно вытянуты, не встаёт.

**Решение:** Перезапустите симуляцию — `Ctrl+C`, затем снова `make gazebo-cpp`.

### Gazebo не открывается / чёрный экран

**Причина:** Проблемы с X11-пробросом в Docker.

**Решение:**
```bash
# На хосте проверьте DISPLAY
echo $DISPLAY  # должно быть :0 или :1
xhost +local:  # разрешить доступ к X-серверу
```

### Docker контейнер не запускается

**Ошибка:** `docker: Error response from daemon: Conflict` или контейнер не найден.

**Решение:**
```bash
make stop     # остановить контейнер
make deploy   # пересобрать и запустить
```

### Ошибка сборки `colcon build`

**Причина:** Отсутствуют зависимости или не установлен `rosdep`.

**Решение:**
```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### RViz показывает "No map received" / карта не загружается

**Причина:** Карта не найдена или AMCL не получил стартовую позу.

**Решение:**
1. Дождитесь полной загрузки симуляции (30-60 секунд)
2. Если через 60 секунд карты нет — установите **2D Pose Estimate** в RViz (кнопка на верхней панели → клик по карте)

---

## ЛР1 — Навигация по waypoint

### "Executor is already spinning" (робот не едет)

**Ошибка в логах:**
```
RuntimeError: Executor is already spinning
```

**Причина:** waypoint_collector пытается запустить spin поверх уже крутящегося executor.

**Решение:** Убедитесь, что используется актуальная версия `waypoint_collector.py`. Ошибка исправлена в текущей версии проекта — если возникает, выполните `make redeploy`.

### Сервис `/start_navigation` не отвечает

**Причина:** Nav2 ещё не загрузился или waypoint_follower не активен.

**Решение:**
```bash
# Проверьте action-сервер
ros2 action list | grep waypoint

# Проверьте, что Nav2 запущен:
ros2 node list | grep nav2
```

### Робот не движется после `make waypoint-start`

**Причины и решения:**

1. **Не установлена начальная поза** — в RViz нажмите **2D Pose Estimate** и укажите положение робота на карте
2. **Waypoint не загружены** — проверьте через `make waypoint-get`
3. **Неправильные координаты** — точка находится в непроходимой зоне (чёрная область на карте)

### Пустые waypoint после загрузки файла

**Ошибка:** `make waypoint-get` показывает пустой список после `make waypoint-load FILE=my_route`.

**Причины и решения:**

1. **Файл не в той директории** — файл должен лежать в `src/gazebo_sim/config/waypoints/`
2. **Некорректный YAML** — проверьте отступы (пробелы, не табуляция)
3. **Нет расширения `.yaml`** — укажите имя файла без расширения

### Робот застревает на препятствии

**Причина:** Costmap не обновляется или препятствие не видно лидару.

**Решение:** Дайте роботу время на перестроение маршрута. Если застрял надолго — остановите навигацию (`make waypoint-stop`), очистите waypoint (`make waypoint-clear`) и попробуйте другой маршрут.

### Эксперимент не запускается — сервис `/start_experiment` не найден

**Ошибка:**
```
ros2 service call /start_experiment std_srvs/Trigger
ERROR: Service '/start_experiment' not found
```

**Причина:** experiment_logger не запущен (возможно, запущена старая версия симуляции без этого узла).

**Решение:** Обновите код и пересоберите пакет:
```bash
cd ~/ws && colcon build --packages-select gazebo_sim
source ~/ws/install/setup.bash
# Перезапустите симуляцию
```

### Результаты эксперимента не сохранились

**Причина:** experiment_logger не был остановлен через `make experiment-stop`, или симуляция была прервана до остановки логгера.

**Решение:** Всегда выполняйте `make experiment-stop` после завершения навигации. Результаты сохраняются в `/tmp/experiments/` внутри контейнера.

### YOLO лог пустой

**Ошибка:** `/tmp/yolo_detections.log` создан, но содержит только заголовки CSV (без данных).

**Причина:** За выбранный интервал YOLO не обнаружил объектов, или камера не публикует изображения.

**Решение:**
```bash
# Проверьте, что камера работает
ros2 topic echo /robot1/color/image_raw --once
# Убедитесь, что в сцене есть объекты для детекции
```

---

## ЛР2 — Детекция YOLO

### YOLO не запускается: "command not found"

**Ошибка:**
```
bash: ros2 run quadropted_perception yolo_detector: command not found
```

**Причина:** Пакет не собран или не установлен.

**Решение:**
```bash
cd ~/ws && colcon build --packages-select quadropted_perception
source ~/ws/install/setup.bash
```

### Нет топика `/detected_image` в RViz

**Причина:** Пустой header у publish-сообщения.

**Решение:** Перезапустите YOLO детектор через `make yolo-detector`. Если не помогает — проверьте, что камера публикует изображение:
```bash
ros2 topic echo /robot1/color/image_raw --once
```

### "cv_bridge" не импортируется

**Ошибка:**
```
ImportError: libcv_bridge.so: cannot open shared object file
```

**Решение:**
```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
```

### "No module named 'ultralytics'"

**Причина:** Ultralytics не установлена в контейнере.

**Решение:**
```bash
pip install --break-system-packages ultralytics
```

### YOLO загружает CPU на 100% — дрейф одометрии

**Причина:** YOLO без троттлинга на CPU загружает все ядра, ROS 2 ноды не успевают обрабатывать данные.

**Решение:**
```bash
# Ограничьте FPS детекции
make yolo-detector FPS=5

# Или используйте лёгкую модель
make yolo-detector MODEL=yolov9t FPS=5
```

### Мало детекций / низкая уверенность

**Решение:** Понизьте порог confidence threshold:
```bash
make yolo-detector CONF=0.3
```

### Модель не найдена

**Ошибка:**
```
Model 'yolov8x.pt' not found
```

**Решение:** Убедитесь, что имя модели указано без расширения (если это кастомная модель) или с правильным именем для Ultralytics (автозагрузка).

### Split-screen в RViz не показывает оба изображения

**Решение:** Проверьте, что в RViz панель **Image** добавлена дважды:
1. Первый — топик `/robot1/color/image_raw`
2. Второй — топик `/detected_image`

---

## Общие ошибки

### Docker контейнер не видит X-сервер

**Ошибка:**
```
Cannot open display
```

**Решение:**
```bash
xhost +local:
```

### Неправильный namespace топиков

Ожидаемые топики имеют префикс `/robot1/`:
- `/robot1/odom` — одометрия
- `/robot1/color/image_raw` — камера
- `/robot1/cmd_vel` — управление

Сервисы waypoint_collector — **без** префикса:
- `/get_waypoints`
- `/start_navigation`
- `/load_waypoints`

### "make" команда не найдена

**Решение:**
```bash
cd WalkingRobotSim
# Команды make выполняются ТОЛЬКО из этой директории
```

### Контейнер не запущен

**Ошибка:**
```
Error: No container found. Run 'make deploy' first.
```

**Решение:**
```bash
make deploy
```

### Файлы не синхронизированы с контейнером

Если вы создали файл на хосте, но он не виден внутри контейнера — убедитесь, что файл находится внутри рабочей директории (`WalkingRobotSim/`). Контейнер монтирует эту папку как `/root/ws`.
