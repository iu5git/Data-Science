# makefiles/simulation.mk

.PHONY: gazebo gazebo-py gazebo-cpp teleop set-pose reset-pose kill-ros exec test-aliases save-logs

## Запуск Gazebo симуляции (Python контроллер)
gazebo:
	$(require-container)
	$(check-x11)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск Gazebo симуляции (ROS $(ROS_DISTRO) + Gazebo Harmonic)...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 launch gazebo_sim launch_python.launch.py \
			use_sim_time:=true gui:=true \
			$(if $(FPS),camera_fps:=${FPS})"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Симуляция завершена, сохранение логов...${NC}\n"
	@$(MAKE) save-logs

## Запуск Gazebo симуляции с Python контроллером
gazebo-py:
	$(require-container)
	$(check-x11)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск Gazebo симуляции с Python контроллером...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 launch gazebo_sim launch_python.launch.py \
			use_sim_time:=true gui:=true \
			$(if $(FPS),camera_fps:=${FPS})"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Симуляция завершена, сохранение логов...${NC}\n"
	@$(MAKE) save-logs

## Запуск Gazebo симуляции с C++ контроллером
gazebo-cpp:
	$(require-container)
	$(check-x11)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск Gazebo симуляции с C++ контроллером...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 launch gazebo_sim launch_cpp.launch.py \
			use_sim_time:=true gui:=true \
			$(if $(FPS),camera_fps:=${FPS})"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Симуляция завершена, сохранение логов...${NC}\n"
	@$(MAKE) save-logs

## Запуск управления роботом (teleop)
teleop:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск управления роботом (ROS $(ROS_DISTRO))...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel"

## Установка положения робота в Gazebo (пример: make set-pose X=1.0 Y=0.0 Z=0.0 YAW=0.0)
set-pose:
	$(require-container)
	@if [ -z "$(X)" ] || [ -z "$(Y)" ] || [ -z "$(Z)" ] || [ -z "$(YAW)" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Укажите все параметры: X Y Z YAW${NC}\n"; \
		printf "Пример: make set-pose X=1.0 Y=0.0 Z=0.0 YAW=0.0\n"; \
		exit 1; \
	fi
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Установка положения робота: X=$(X) Y=$(Y) Z=$(Z) YAW=$(YAW)${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		gz service -s /world/default/set_pose \
			--reqtype gz.msgs.Pose \
			--reptype gz.msgs.Boolean \
			--timeout 1000 \
			--req \"name: 'go2', position: {x: $(X), y: $(Y), z: $(Z)}, orientation: {z: $(YAW)}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Положение установлено${NC}\n"

## Сброс положения робота в начало (0, 0, 0.5, 0)
reset-pose:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Сброс положения робота в начало...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		gz service -s /world/default/set_pose \
			--reqtype gz.msgs.Pose \
			--reptype gz.msgs.Boolean \
			--timeout 1000 \
			--req \"name: 'go2', position: {x: 0, y: 0, z: 0.5}, orientation: {z: 0}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Положение сброшено${NC}\n"

## Выполнение команды в контейнере (пример: make exec CMD="ros2 topic list")
exec:
	$(require-container)
	@if [ -z "$(CMD)" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Укажите команду для выполнения${NC}\n" >&2; \
		printf "Пример: make exec CMD='ros2 topic list'\n" >&2; \
		exit 1; \
	fi
	@docker exec -i $(CONTAINER_NAME) bash -c "source /opt/ros/$(ROS_DISTRO)/setup.bash && source /root/ws/install/setup.bash && $(CMD)"

## Проверка алиасов в контейнере
test-aliases:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Проверка алиасов в контейнере...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		source ~/.bashrc && \
		echo 'Проверка алиасов:' && \
		alias topics && \
		echo 'Топики (первые 3):' && \
		topics | head -3 && \
		echo 'Алиасы работают!'"

## Очистка всех ROS/Gazebo процессов в контейнере
kill-ros:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Очистка всех ROS/Gazebo процессов...${NC}\n"
	@if docker ps --format '{{.Names}}' | grep -q $(CONTAINER_NAME); then \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Убиваем ROS/Gazebo процессы в контейнере...${NC}\n"; \
		docker exec -it $(CONTAINER_NAME) bash -c "\
			pkill -f 'ros2\|gz sim\|rviz2\|gazebo' || true; \
			pkill -f 'robot_controller\|quadruped\|teleop' || true; \
			pkill -f 'python.*robot\|python.*controller' || true; \
			pkill -f '/robot1/' || true; \
			pkill -f 'cmd_vel\|joint_states\|imu_plugin' || true; \
			rm -f /tmp/ros* 2>/dev/null || true; \
			rm -f ~/.ros/* 2>/dev/null || true; \
			pkill -f 'gz-' || true; \
			pkill -f 'ign-' || true; \
			sleep 2; \
			if pgrep -f 'ros2\|gz sim\|rviz2' > /dev/null; then \
				printf '${YELLOW}${BOLD}[!]${NC} ${YELLOW}Некоторые ROS процессы все еще запущены${NC}\n'; \
				pgrep -f 'ros2\|gz sim\|rviz2' || true; \
			else \
				printf '${GREEN}${BOLD}[v]${NC} ${GREEN}Все ROS/Gazebo процессы успешно остановлены${NC}\n'; \
			fi"; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Контейнер $(CONTAINER_NAME) не запущен${NC}\n"; \
	fi

## Сохранение логов Gazebo сессии
save-logs:
	@if docker ps --format '{{.Names}}' | grep -q $(CONTAINER_NAME); then \
		printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Сохранение логов сессии Gazebo...${NC}\n"; \
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
				echo "Время: $$(date)" >> "$$merged_file"; \
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
		cd $(DOCKER_DIR) && $(COMPOSE) logs --no-color > "$$backup_folder/docker_compose.log" 2>/dev/null || true && \
		echo "=== Логи сессии Walking Robot Simulator ===" > "$$backup_folder/session_info.log" && \
		echo "Время: $$(date)" >> "$$backup_folder/session_info.log" && \
		echo "Тип: Gazebo симуляция" >> "$$backup_folder/session_info.log" && \
		echo "Хост: $$hostname" >> "$$backup_folder/session_info.log" && \
		echo "Контейнер: $(CONTAINER_NAME)" >> "$$backup_folder/session_info.log" && \
		cd "$(PROJECT_ROOT)"; \
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
		printf "Проверьте: $$backup_folder/merged_logs/\n"; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Контейнер не запущен, логи не сохранены${NC}\n"; \
	fi
