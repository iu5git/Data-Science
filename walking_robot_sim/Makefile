# Walking Robot Simulation - Makefile
# Конфигурация + include модулей

SHELL := /bin/bash

# ════════════════════════════════════════════════════════════
# КОНФИГУРАЦИЯ
# ════════════════════════════════════════════════════════════

CONTAINER_NAME  := walking_robot_sim
IMAGE_NAME      := walking_robot_sim:latest
DOCKER_DIR      := $(CURDIR)/src/docker
PROJECT_ROOT    := $(CURDIR)
ROS_DISTRO      := jazzy
COMPOSE         := docker compose
COMPOSE_FILE    := $(DOCKER_DIR)/compose.yml

# ════════════════════════════════════════════════════════════
# ЦВЕТА
# ════════════════════════════════════════════════════════════

BLUE    := \033[0;34m
GREEN   := \033[0;32m
YELLOW  := \033[1;33m
RED     := \033[0;31m
CYAN    := \033[0;36m
BOLD    := \033[1m
NC      := \033[0m

# ════════════════════════════════════════════════════════════
# HELPER
# ════════════════════════════════════════════════════════════

# Проверка что контейнер запущен
define require-container
	@if ! docker ps --format '{{.Names}}' | grep -q $(CONTAINER_NAME); then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Контейнер $(CONTAINER_NAME) не запущен.${NC}\n" >&2; \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Запустите: make deploy${NC}\n" >&2; \
		exit 1; \
	fi
endef

# Проверка и настройка X11 для GUI
define check-x11
	@if [ -z "$$DISPLAY" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}DISPLAY не установлен.${NC}\n" >&2; \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Установите: export DISPLAY=:0${NC}\n" >&2; \
		exit 1; \
	fi
	@xhost +local:root >/dev/null 2>&1 || true
	@xhost +local:$(USER) >/dev/null 2>&1 || true
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}X11 настроен: DISPLAY=$$DISPLAY${NC}\n"
endef

# ════════════════════════════════════════════════════════════
# ПОДКЛЮЧЕНИЕ МОДУЛЕЙ
# ════════════════════════════════════════════════════════════

include makefiles/help.mk
include makefiles/docker.mk
include makefiles/simulation.mk
include makefiles/controller.mk
include makefiles/navigation.mk
include makefiles/yolo.mk
include makefiles/experiment.mk
include makefiles/ci.mk
include makefiles/test.mk

# ════════════════════════════════════════════════════════════
# DEFAULT TARGET
# ════════════════════════════════════════════════════════════

.DEFAULT_GOAL := help
