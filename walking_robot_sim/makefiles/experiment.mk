# makefiles/experiment.mk

.PHONY: experiment-start experiment-stop experiment-result

## Запустить эксперимент (логгирование времени и дистанции)
experiment-start:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск эксперимента...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /start_experiment std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Эксперимент запущен${NC}\n"

## Остановить эксперимент и сохранить результаты
experiment-stop:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Остановка эксперимента...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /stop_experiment std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Результаты сохранены${NC}\n"

## Скопировать результаты эксперимента на хост
experiment-result:
	$(require-container)
	@mkdir -p experiments
	@docker cp $(CONTAINER_NAME):/tmp/experiments/. experiments/ 2>/dev/null || true
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Результаты скопированы в experiments/${NC}\n"
	@ls -la experiments/

## Полный цикл: загрузить маршрут + эксперимент + старт навигации
experiment-run:
	$(require-container)
	@if [ -z "$(FILE)" ]; then \
		printf "${RED}${BOLD}[x]${NC} ${RED}Укажите файл маршрута: make experiment-run FILE=my_route${NC}\n"; \
		exit 1; \
	fi
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Загрузка маршрута $(FILE)...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /load_waypoints quadropted_msgs/srv/LoadWaypoints \"{file_path: '$(FILE)'}\""
	@sleep 1
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск эксперимента...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /start_experiment std_srvs/Trigger"
	@sleep 1
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Старт навигации...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 service call /start_navigation std_srvs/Trigger"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Эксперимент запущен! Дождитесь завершения навигации в RViz.${NC}\n"
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}После завершения выполните: make experiment-stop${NC}\n"
