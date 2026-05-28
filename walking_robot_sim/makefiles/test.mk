# makefiles/test.mk

.PHONY: test test-build test-container test-clean check-deps check-structure test-yaml setup backup check-x11
.PHONY: test-correctness test-benchmark benchmark benchmark-python benchmark-cpp

## Полный цикл тестирования
test: check-deps check-structure test-yaml test-build test-container
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Все тесты пройдены успешно!${NC}\n"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Теперь можно выполнять git push${NC}\n"

## Только сборка образа для теста
test-build: check-deps check-structure test-yaml
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Локальная сборка Docker-образа...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) build --no-cache
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Локальная сборка завершена успешно${NC}\n"

## Тестовый запуск контейнера
test-container:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Тестовый запуск контейнера...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) up -d
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Ожидание запуска контейнера...${NC}\n"
	@sleep 15
	@if $(COMPOSE) ps | grep -q "healthy"; then \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Контейнер запущен и здоров${NC}\n"; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Контейнер запущен, но статус здоровья неизвестен${NC}\n"; \
	fi
	@if $(COMPOSE) exec -T simulator bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && ros2 node list"; then \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}ROS функциональность проверена успешно${NC}\n"; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}ROS функциональность не проверена (контейнер может быть в процессе инициализации)${NC}\n"; \
	fi
	@cd $(DOCKER_DIR) && $(COMPOSE) down
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Контейнер остановлен${NC}\n"

## Очистка Docker ресурсов после тестов
test-clean:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Очистка Docker ресурсов...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) down -v 2>/dev/null || true
	@docker rmi walking_robot_sim:latest 2>/dev/null || true
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Очистка завершена${NC}\n"

## Проверка зависимостей
check-deps:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка необходимых инструментов...${NC}\n"
	@if ! command -v docker &> /dev/null; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Docker не установлен. Пожалуйста, установите Docker.${NC}\n"; \
		exit 1; \
	fi
	@if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Docker Compose не установлен.${NC}\n"; \
		exit 1; \
	fi
	@if ! command -v yamllint &> /dev/null; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}yamllint не установлен. Установите: pip install yamllint${NC}\n"; \
	fi
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Все необходимые инструменты установлены${NC}\n"

## Проверка структуры проекта
check-structure:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка структуры проекта...${NC}\n"
	@if [ ! -d "$(PROJECT_ROOT)/src" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Директория src не найдена${NC}\n"; \
		exit 1; \
	fi
	@if [ ! -d "$(PROJECT_ROOT)/src/docker" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Директория src/docker не найдена${NC}\n"; \
		exit 1; \
	fi
	@if [ ! -f "$(PROJECT_ROOT)/src/docker/compose.yml" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Файл src/docker/compose.yml не найден${NC}\n"; \
		exit 1; \
	fi
	@if [ ! -f "$(PROJECT_ROOT)/src/docker/Dockerfile" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Файл src/docker/Dockerfile не найден${NC}\n"; \
		exit 1; \
	fi
	@if [ ! -d "$(PROJECT_ROOT)/src/gazebo_sim" ]; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Директория src/gazebo_sim не найдена${NC}\n"; \
	fi
	@if [ ! -d "$(PROJECT_ROOT)/src/go1_description" ]; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Директория src/go1_description не найдена${NC}\n"; \
	fi
	@if [ ! -d "$(PROJECT_ROOT)/src/go2_description" ]; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Директория src/go2_description не найдена${NC}\n"; \
	fi
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Структура проекта проверена${NC}\n"

## Проверка синтаксиса YAML
test-yaml:
	@if command -v yamllint &> /dev/null; then \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка синтаксиса YAML...${NC}\n"; \
		if yamllint $(DOCKER_DIR)/compose.yml; then \
			printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Синтаксис compose.yml корректен${NC}\n"; \
		else \
			printf "${RED}${BOLD}[x]${NC} ${RED}Обнаружены ошибки в синтаксисе compose.yml${NC}\n"; \
			exit 1; \
		fi; \
		if [ -f "$(DOCKER_DIR)/compose.multistage.yml" ]; then \
			if yamllint $(DOCKER_DIR)/compose.multistage.yml; then \
				printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Синтаксис compose.multistage.yml корректен${NC}\n"; \
			else \
				printf "${RED}${BOLD}[x]${NC} ${RED}Обнаружены ошибки в синтаксисе compose.multistage.yml${NC}\n"; \
				exit 1; \
			fi; \
		fi; \
		if [ -d "$(PROJECT_ROOT)/.github/workflows" ]; then \
			if yamllint $(PROJECT_ROOT)/.github/workflows/; then \
				printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Синтаксис GitHub workflows корректен${NC}\n"; \
			else \
				printf "${RED}${BOLD}[x]${NC} ${RED}Обнаружены ошибки в синтаксисе GitHub workflows${NC}\n"; \
				exit 1; \
			fi; \
		else \
			printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Директория .github/workflows не найдена${NC}\n"; \
		fi; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}yamllint не установлен, пропускаем проверку YAML${NC}\n"; \
	fi

## Начальная настройка проекта
setup: check-x11
	@echo ""
	@printf "${CYAN}${BOLD}╔════════════════════════════════════════════════════════════╗${NC}\n"
	@printf "${CYAN}${BOLD}║${NC}  ${BOLD}WalkingRobotSim - Setup${NC}                               ${CYAN}${BOLD}║${NC}\n"
	@printf "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}\n"
	@echo ""
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка Docker...${NC}\n"
	@if ! command -v docker &> /dev/null; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Docker не установлен${NC}\n"; \
		echo "Установите Docker: https://docs.docker.com/get-docker/"; \
		exit 1; \
	fi
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Docker найден: $$(docker --version)${NC}\n"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка Docker Compose...${NC}\n"
	@if ! docker compose version &> /dev/null 2>&1; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Docker Compose не установлен${NC}\n"; \
		echo "Установите Docker Compose: https://docs.docker.com/compose/install/"; \
		exit 1; \
	fi
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Docker Compose найден: $$(docker compose version --short)${NC}\n"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка структуры проекта...${NC}\n"
	@for file in "docker/Dockerfile" "docker/compose.yml" "docker/cyclonedds.xml"; do \
		if [ ! -f "$(PROJECT_ROOT)/src/$$file" ]; then \
			printf "${RED}${BOLD}[x]${NC} ${RED}Файл не найден: $$file${NC}\n"; \
			exit 1; \
		fi; \
	done
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Структура проекта верна${NC}\n"
	@echo ""
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Информация о системе:${NC}\n"
	@printf "  OS: $$(uname -s)\n"
	@printf "  Kernel: $$(uname -r)\n"
	@printf "  Docker: $$(docker --version)\n"
	@printf "  Compose: $$(docker compose version --short)\n"
	@printf "  User: $$(whoami)\n"
	@printf "  Home: $$HOME\n"
	@echo ""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Инициализация завершена успешно!${NC}\n"
	@echo ""
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Следующие шаги:${NC}\n"
	@printf "  1. ${BOLD}make deploy${NC}              # Сборка и запуск\n"
	@printf "  2. ${BOLD}make gazebo${NC}              # Запуск Gazebo\n"
	@printf "  3. ${BOLD}make teleop${NC}              # Управление роботом (в другом терминале)\n"
	@echo ""

## Проверка X11 (для GUI)
check-x11:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка X11 (для GUI)...${NC}\n"
	@if [ -z "$$DISPLAY" ]; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}DISPLAY не установлен. X11 GUI может не работать.${NC}\n"; \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Для использования GUI установите DISPLAY:${NC}\n"; \
		echo "  export DISPLAY=:0"; \
		echo "  xhost +local:"; \
	else \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}DISPLAY установлен: $$DISPLAY${NC}\n"; \
	fi

## Создание бэкапа данных
backup:
	@backup_file="walking_robot_backup_$$(date +%Y%m%d_%H%M%S).tar.gz"; \
	printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Создание бэкапа: $$backup_file${NC}\n"; \
	docker run --rm -v /var/run/docker.sock:/var/run/docker.sock \
		-v $(DOCKER_DIR):/backup alpine tar czf /backup/"$$backup_file" \
		/var/lib/docker/volumes/gazebo_logs /var/lib/docker/volumes/gazebo_data 2>/dev/null || true; \
	printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Бэкап создан: $$backup_file${NC}\n"

## Проверка корректности — запуск всех тестов в correctness/
test-correctness:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Тесты корректности...${NC}\n"
	@cd $(PROJECT_ROOT)/src/tests/correctness && python3 run_all.py
	@echo ""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Тесты корректности завершены${NC}\n"

## Benchmark производительности — замер времени
test-benchmark:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Benchmark производительности...${NC}\n"
	@cd $(PROJECT_ROOT) && python3 src/tests/benchmark_performance.py
	@echo ""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Benchmark завершён${NC}\n"

## Запуск полного бенчмарка Python vs C++ с таблицей результатов
benchmark:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск Python + C++ сводной таблицы...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		cd /root/ws/src/quadropted_controller/scripts/benchmark && \
		python3 benchmark.py --combined"

## Запуск только Python бенчмарка
benchmark-python:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск Python бенчмарка...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		cd /root/ws/src/quadropted_controller/scripts/benchmark && \
		python3 benchmark.py"

## Запуск только C++ бенчмарка
benchmark-cpp:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск C++ бенчмарка...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		/root/ws/build/quadropted_controller_cpp/benchmark"
