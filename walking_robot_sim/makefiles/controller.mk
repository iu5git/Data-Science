# makefiles/controller.mk

.PHONY: rest trot crawl stand

## Перевести робота в режим REST (отдых)
rest:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Перевод робота в режим REST...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 topic pub --once /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand \"{mode: REST, robot_id: 1}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Режим REST установлен${NC}\n"

## Перевести робота в режим TROT (бег рысью)
trot:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Перевод робота в режим TROT...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 topic pub --once /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand \"{mode: TROT, robot_id: 1}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Режим TROT установлен${NC}\n"

## Перевести робота в режим CRAWL (ползание)
crawl:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Перевод робота в режим CRAWL...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 topic pub --once /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand \"{mode: CRAWL, robot_id: 1}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Режим CRAWL установлен${NC}\n"

## Перевести робота в режим STAND (стойка)
stand:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Перевод робота в режим STAND...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash; \
		source /root/ws/install/setup.bash 2>/dev/null || true; \
		ros2 topic pub --once /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand \"{mode: STAND, robot_id: 1}\""
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Режим STAND установлен${NC}\n"
