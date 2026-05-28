# Walking Robot Simulator

Симулятор четвероногого робота на базе ROS 2 Jazzy с Gazebo Sim, Docker-контейнеризацией, Nav2 навигацией и YOLO детекцией.

## Лабораторные работы

- [Работа №0: Установка, сборка и запуск](exercise/lab0-introduction.md)
- [Работа №1: Автономная навигация по путевым точкам](exercise/lab1-waypoint.md)
- [Работа №2: Детекция объектов YOLO](exercise/lab2-yolo.md)
- [Типовые ошибки](exercise/common-errors-lab.md)
- [Результаты экспериментов](experiments/)

## Быстрый старт

```bash
make deploy       # сборка Docker образа + запуск
make gazebo-cpp   # запуск симуляции с C++ контроллером
```

**Требования:** Docker 20.10+, Docker Compose v2, Linux с X11, 8GB+ RAM.

## Оригинальный репозиторий

[github.com/RedAlexDad/WalkingRobotSim](https://github.com/RedAlexDad/WalkingRobotSim) — тег `v.0.0.4` (актуальная версия для лабораторных работ).
