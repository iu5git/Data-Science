# makefiles/yolo.mk

.PHONY: yolo-detector yolo-experiment-start yolo-experiment-stop yolo-experiment-result yolo-visualizer

## Запуск YOLO детектора (инференс, вывод логов)
yolo-detector:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск YOLO детектора...${NC}\n"
	@docker exec -it $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		ros2 run quadropted_perception yolo_detector \
			$(if $(or $(MODEL),$(FPS),$(CONF)),--ros-args) \
			$(if $(MODEL),-p model:=${MODEL}) \
			$(if $(FPS),-p fps:=${FPS}) \
			$(if $(CONF),-p confidence_threshold:=${CONF})" || true

## Запустить YOLO эксперимент (логгирование в фоне, пример: LOG_INTERVAL=10)
yolo-experiment-start:
	$(require-container)
	@STAMP=$$(date +%Y%m%d_%H%M%S); \
	LOG_FILE="/tmp/yolo_experiment_$$STAMP.log"; \
	printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск YOLO эксперимента (интервал: $(or $(LOG_INTERVAL),10) сек)...${NC}\n" && \
	docker exec -d $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		ros2 run quadropted_perception yolo_detector \
			--ros-args \
			-p log_interval_sec:=${or $(LOG_INTERVAL),10} \
			-p log_file:=/tmp/yolo_experiment_$$STAMP.log" && \
	echo $$STAMP | docker exec -i $(CONTAINER_NAME) bash -c "cat > /tmp/.yolo_experiment_stamp" && \
	sleep 2 && \
	printf "${GREEN}${BOLD}[v]${NC} ${GREEN}YOLO эксперимент запущен (лог: yolo_experiment_$$STAMP.log)${NC}\n"

## Остановить YOLO эксперимент и сохранить результат
yolo-experiment-stop:
	$(require-container)
	@STAMP=$$(docker exec $(CONTAINER_NAME) bash -c "cat /tmp/.yolo_experiment_stamp 2>/dev/null || true"); \
	LOG_FILE="yolo_experiment_$$STAMP.log"; \
	printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Остановка YOLO эксперимента...${NC}\n" && \
	docker exec $(CONTAINER_NAME) bash -c "pkill -f 'yolo_experiment' 2>/dev/null; true" 2>/dev/null || true && \
	sleep 1 && \
	if [ -n "$$STAMP" ]; then \
		mkdir -p experiments && \
		docker cp $(CONTAINER_NAME):/tmp/$$LOG_FILE experiments/ 2>/dev/null || true; \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Результат: experiments/$$LOG_FILE${NC}\n"; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Нет активного эксперимента${NC}\n"; \
	fi

## Скопировать лог YOLO эксперимента на хост
yolo-experiment-result:
	$(require-container)
	@mkdir -p experiments; \
	STAMP=$$(docker exec $(CONTAINER_NAME) bash -c "cat /tmp/.yolo_experiment_stamp 2>/dev/null || true"); \
	if [ -n "$$STAMP" ]; then \
		LOG_FILE="yolo_experiment_$$STAMP.log"; \
		docker cp $(CONTAINER_NAME):/tmp/$$LOG_FILE experiments/ 2>/dev/null && \
			printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Лог скопирован в experiments/$$LOG_FILE${NC}\n" || \
			printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Файл /tmp/$$LOG_FILE не найден в контейнере${NC}\n"; \
		ls -la experiments/$$LOG_FILE 2>/dev/null || true; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Нет информации об активном эксперименте${NC}\n"; \
		ls -la experiments/yolo_experiment_*.log 2>/dev/null || printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}Нет файлов yolo_experiment_*.log${NC}\n"; \
	fi

## Запуск визуализации детекций: RViz + visualizer (split: raw / detected)
yolo-visualizer:
	$(require-container)
	$(check-x11)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск визуализации детекций...${NC}\n"
	@docker exec -d $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		ros2 run quadropted_perception visualizer"
	@sleep 1
	@docker exec -d $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		rviz2 -d /root/ws/src/quadropted_perception/rviz/yolo_detection.rviz"
