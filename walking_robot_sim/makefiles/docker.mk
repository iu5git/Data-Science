# makefiles/docker.mk

.PHONY: deploy build up up-bg down restart clean status logs shell deploy-no-cache build-stage build-stage-list

## Сборка и запуск контейнера (рекомендуется)
deploy: build up

## Сборка и запуск контейнера без кэша
deploy-no-cache: build-no-cache up

## Сборка Docker образа
build:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Сборка Docker образа с кэшированием по этапам...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) --progress=auto build
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Образ собран${NC}\n"

## Сборка Docker образа без кэша
build-no-cache:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Сборка Docker образа БЕЗ кэширования...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) --progress=auto build --no-cache
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Образ собран без кэша${NC}\n"

## Запуск контейнера
up:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск контейнера $(CONTAINER_NAME)...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) up -d
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Ожидание инициализации ROS окружения...${NC}\n"
	@attempt=0; \
	while [ $$attempt -lt 30 ]; do \
		if docker exec $(CONTAINER_NAME) bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && source /root/ws/install/setup.bash 2>/dev/null && ros2 node list" >/dev/null 2>&1; then \
			printf "${GREEN}${BOLD}[v]${NC} ${GREEN}ROS окружение готово ($${attempt} сек)${NC}\n"; \
			break; \
		fi; \
		attempt=$$((attempt + 1)); \
		sleep 1; \
		printf "."; \
	done; \
	if [ $$attempt -eq 30 ]; then \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}ROS окружение может быть не готово, но продолжаем...${NC}\n"; \
	fi
	@echo ""
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Статус контейнера:${NC}\n"
	@docker ps --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Контейнер запущен${NC}\n"

## Запуск контейнера в фоновом режиме (без ожидания ROS)
up-bg:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск контейнера $(CONTAINER_NAME) в фоновом режиме...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) up -d
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Контейнер запущен${NC}\n"

## Остановка контейнера с сохранением логов
down:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Остановка контейнера $(CONTAINER_NAME)...${NC}\n"
	@if docker ps --format '{{.Names}}' | grep -q $(CONTAINER_NAME); then \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Сохранение логов сессии...${NC}\n"; \
		timestamp=$$(date +%s); \
		hostname=$$(hostname); \
		backup_folder="logs/gazebo_backup_$${timestamp}_$${hostname}"; \
		gazebo_folder="logs/gazebo"; \
		mkdir -p "$$backup_folder" 2>/dev/null || { \
			printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Не удалось создать $$backup_folder, используем /tmp/${NC}\n"; \
			backup_folder="/tmp/gazebo_backup_$${timestamp}_$${hostname}"; \
			mkdir -p "$$backup_folder"; \
		}; \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Копирование ROS логов из контейнера...${NC}\n"; \
		docker cp $(CONTAINER_NAME):/root/ws/logs/. "$$backup_folder/" 2>/dev/null || true; \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Объединение логов по типам...${NC}\n"; \
		cd "$$backup_folder" && \
		mkdir -p merged_logs && \
		for pattern in "amcl" "behavior_server" "bt_navigator" "controller_server" "ekf_node" "gz sim server" "image_bridge" "lifecycle_manager" "map_server" "parameter_bridge" "planner_server" "python3" "robot_state_publisher" "rviz2" "smoother_server"; do \
			files=$$(ls $${pattern}_*.log 2>/dev/null || true); \
			if [ -n "$$files" ]; then \
				mkdir -p "$$pattern"; \
				merged_file="merged_logs/$${pattern}_combined.log"; \
				echo "=== Объединенные логи $${pattern} ===" > "$$merged_file"; \
				echo "Время создания: $$(date)" >> "$$merged_file"; \
				echo "" >> "$$merged_file"; \
				for file in $$files; do \
					if [ -f "$$file" ]; then \
						mv "$$file" "$$pattern/"; \
						echo "" >> "$$merged_file"; \
						echo "=== Файл: $$pattern/$$(basename $$file) ===" >> "$$merged_file"; \
						cat "$$pattern/$$(basename $$file)" >> "$$merged_file"; \
						echo "" >> "$$merged_file"; \
					fi; \
				done; \
				echo "Объединен: $$pattern ($$(echo $$files | wc -w) файлов)"; \
			fi; \
		done && \
		if [ -d "../$$gazebo_folder" ]; then \
			cp -r ../"$$gazebo_folder"/* "./" 2>/dev/null || true; \
		fi && \
		$(COMPOSE) logs --no-color > "docker_compose.log" 2>/dev/null || true && \
		echo "=== Логи сессии Walking Robot Simulator ===" > "session_info.log" && \
		echo "Время: $$(date)" >> "session_info.log" && \
		echo "Хост: $$hostname" >> "session_info.log" && \
		echo "Контейнер: $(CONTAINER_NAME)" >> "session_info.log" && \
		cd "../.."; \
		if [ -d "$$gazebo_folder" ]; then \
			printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Очистка папки gazebo...${NC}\n"; \
			docker run --rm -v "$$(pwd)/$$gazebo_folder":/tmp/clean alpine sh -c "rm -rf /tmp/clean/*" 2>/dev/null || true; \
		fi; \
		mkdir -p "$$gazebo_folder"; \
		file_count=$$(find "$$backup_folder" -type f 2>/dev/null | wc -l); \
		merged_count=$$(find "$$backup_folder/merged_logs" -type f 2>/dev/null | wc -l); \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Логи сохранены: $$backup_folder${NC}\n"; \
		printf "Всего файлов: $$file_count\n"; \
		printf "Объединенных логов: $$merged_count\n"; \
	fi
	@cd $(DOCKER_DIR) && $(COMPOSE) down
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Контейнер остановлен${NC}\n"

## Перезапуск контейнера
restart: down up

## Полная очистка Docker образов и контейнеров
clean:
	@printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Очистка Docker образов и контейнеров...${NC}\n"
	@cd $(DOCKER_DIR) && $(COMPOSE) down -v --remove-orphans
	@docker system prune -f
	@docker volume prune -f
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Очистка завершена${NC}\n"

## Статус контейнера
status:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Статус контейнера:${NC}\n"
	@docker ps --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}"
	@echo ""
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Использование ресурсов:${NC}\n"
	@docker stats --no-stream --format "table {{.Container}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}" $(CONTAINER_NAME) 2>/dev/null || printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Контейнер не запущен${NC}\n"

## Просмотр логов контейнера
logs:
	@cd $(DOCKER_DIR) && $(COMPOSE) logs -f

## Подключение к контейнеру (shell)
shell:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Подключение к контейнеру $(CONTAINER_NAME)...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		echo 'alias sim=\"ros2 launch gazebo_sim launch_cpp.launch.py use_sim_time:=true gui:=true\"' >> ~/.bashrc && \
		echo 'alias teleop=\"ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel\"' >> ~/.bashrc && \
		echo 'alias topics=\"ros2 topic list\"' >> ~/.bashrc && \
		echo 'alias nodes=\"ros2 node list\"' >> ~/.bashrc && \
		echo 'alias services=\"ros2 service list\"' >> ~/.bashrc && \
		echo 'alias actions=\"ros2 action list\"' >> ~/.bashrc && \
		echo 'alias waypoints=\"ros2 service call /robot1/get_waypoints quadropted_msgs/srv/GetWaypoints\"' >> ~/.bashrc && \
		echo 'alias nav-start=\"ros2 service call /robot1/start_navigation std_srvs/srv/Trigger\"' >> ~/.bashrc && \
		echo 'alias nav-stop=\"ros2 service call /robot1/stop_navigation std_srvs/srv/Trigger\"' >> ~/.bashrc && \
		echo 'alias nav-clear=\"ros2 service call /robot1/clear_waypoints std_srvs/srv/Trigger\"' >> ~/.bashrc && \
		echo 'alias detect=\"ros2 launch quadropted_perception yolo_detector.launch.py\"' >> ~/.bashrc && \
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		export PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\[\033[01;31m\](ROS $(ROS_DISTRO))\[\033[00m\]\$$ ' && \
		printf '${GREEN}${BOLD}ROS $(ROS_DISTRO) окружение настроено!${NC}\n' && \
		printf '${CYAN}Доступные команды:${NC}\n' && \
		echo '   sim          - Запуск Gazebo симуляции (в контейнере)' && \
		echo '   teleop       - Управление роботом с клавиатуры' && \
		echo '   topics       - ros2 topic list' && \
		echo '   nodes        - ros2 node list' && \
		echo '   services     - ros2 service list' && \
		echo '   actions      - ros2 action list' && \
		echo '   waypoints    - Показать текущие путевые точки' && \
		echo '   nav-start    - Запустить навигацию по waypoint' && \
		echo '   nav-stop     - Остановить навигацию' && \
		echo '   nav-clear    - Очистить waypoint' && \
		echo '   detect       - Запустить YOLO детектор' && \
		source ~/.bashrc && \
		exec bash"

## Сборка конкретного этапа Docker (пример: make build-stage STAGE=ros-core)
build-stage:
	@if [ -z "$(STAGE)" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Укажите этап: make build-stage STAGE=<stage>${NC}\n"; \
		echo "Доступные этапы: base-system ros-core ros-control ros-simulation ros-navigation ros-vision ros-tools python-deps workspace final"; \
		exit 1; \
	fi
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Сборка этапа: $(STAGE)${NC}\n"
	@cd $(DOCKER_DIR) && docker build \
		--target $(STAGE) \
		--tag walking_robot_sim:$(STAGE) \
		--tag walking_robot_sim:latest \
		--cache-from walking_robot_sim:$(STAGE) \
		--cache-from walking_robot_sim:latest \
		.
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Этап $(STAGE) собран${NC}\n"

## Показать доступные этапы сборки
build-stage-list:
	@printf "${CYAN}Доступные этапы сборки:${NC}\n"
	@printf "  ${BOLD}base-system${NC}     - Системные зависимости\n"
	@printf "  ${BOLD}ros-core${NC}        - ROS Core пакеты\n"
	@printf "  ${BOLD}ros-control${NC}     - ROS Control пакеты\n"
	@printf "  ${BOLD}ros-simulation${NC}  - Gazebo и simulation\n"
	@printf "  ${BOLD}ros-navigation${NC}  - Navigation пакеты\n"
	@printf "  ${BOLD}ros-vision${NC}      - Vision и sensor пакеты\n"
	@printf "  ${BOLD}ros-tools${NC}       - Tools и утилиты\n"
	@printf "  ${BOLD}python-deps${NC}     - Python зависимости\n"
	@printf "  ${BOLD}workspace${NC}       - Сборка workspace\n"
	@printf "  ${BOLD}final${NC}           - Финальный образ (по умолчанию)\n"
