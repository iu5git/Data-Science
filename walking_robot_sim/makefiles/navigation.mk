# makefiles/navigation.mk

.PHONY: waypoint-start waypoint-clear waypoint-navigate waypoint-stop waypoint-resume waypoint-load waypoint-get

## Запустить навигацию по всем waypoints (сервис /start_navigation)
waypoint-start:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск навигации по waypoints...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /start_navigation std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Команда отправлена${NC}\n"

## Очистить все waypoints (сервис /clear_waypoints)
waypoint-clear:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Очистка waypoints...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /clear_waypoints std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Команда отправлена${NC}\n"

## Навигация к конкретному waypoint по индексу (пример: make waypoint-navigate INDEX=2)
waypoint-navigate:
	$(require-container)
	@if [ -z "$(INDEX)" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Укажите индекс: make waypoint-navigate INDEX=2${NC}\n"; \
		exit 1; \
	fi
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Навигация к waypoint $(INDEX)...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /navigate_to_waypoint quadropted_msgs/srv/WaypointNavigate \"{index: $(INDEX)}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Команда отправлена${NC}\n"

## Остановить текущую навигацию (сервис /stop_navigation)
waypoint-stop:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Остановка навигации...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /stop_navigation std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Команда отправлена${NC}\n"

## Продолжить навигацию с прерванного waypoint (сервис /resume_navigation)
waypoint-resume:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Продолжение навигации...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /resume_navigation std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Команда отправлена${NC}\n"

## Загрузить waypoints из JSON-файла (пример: make waypoint-load FILE=test.json)
waypoint-load:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Загрузка waypoints..."
ifneq ($(FILE),)
	@printf " из $(FILE)...${NC}\n"
else
	@printf " (по умолчанию)...${NC}\n"
endif
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /load_waypoints quadropted_msgs/srv/LoadWaypoints \"{file_path: '$(FILE)'}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Команда отправлена${NC}\n"

## Получить текущие waypoints (сервис /get_waypoints)
waypoint-get:
	$(require-container)
	@docker exec $(CONTAINER_NAME) bash -c 'source /opt/ros/$(ROS_DISTRO)/setup.bash; source /root/ws/install/setup.bash 2>/dev/null || true; ros2 service call /get_waypoints quadropted_msgs/srv/GetWaypoints "{}" 2>/dev/null' | python3 -c "import sys,re; d=sys.stdin.read(); m=re.findall(r'x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+),\s*yaw=([-\d.]+)',d); [print(str(i)+chr(9)+str(round(float(x),3))+chr(9)+str(round(float(y),3))+chr(9)+str(round(float(z),3))+chr(9)+str(round(float(yaw),3))) for i,(x,y,z,yaw) in enumerate(m)]"
