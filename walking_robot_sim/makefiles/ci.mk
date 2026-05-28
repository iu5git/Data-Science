# makefiles/ci.mk

.PHONY: ci-lint ci-test ci-lint-yaml ci-lint-python ci-lint-cpp ci-test-cpp

## Полный CI lint check (YAML + Python + C++)
ci-lint: ci-lint-yaml ci-lint-python ci-lint-cpp
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Все lint проверки пройдены${NC}\n"

## YAML lint (yamllint)
ci-lint-yaml:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}YAML lint (yamllint)...${NC}\n"
	@if command -v yamllint &> /dev/null; then \
		yamllint -c .yamllint .github/workflows/ && \
		yamllint -c .yamllint src/docker/*.yml && \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}YAML lint OK${NC}\n"; \
	else \
		pip install yamllint -q && \
		yamllint -c .yamllint .github/workflows/ && \
		yamllint -c .yamllint src/docker/*.yml && \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}YAML lint OK${NC}\n"; \
	fi

## Python lint (ruff)
ci-lint-python:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Python lint (ruff)...${NC}\n"
	@if command -v ruff &> /dev/null; then \
		ruff check src/ && \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Python lint OK${NC}\n"; \
	else \
		pip install ruff -q && \
		ruff check src/ && \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Python lint OK${NC}\n"; \
	fi

## C++ format check (clang-format)
ci-lint-cpp:
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}C++ format check (clang-format)...${NC}\n"
	@if command -v clang-format &> /dev/null; then \
		find src/quadropted_controller_cpp -name '*.hpp' -o -name '*.cpp' | \
			xargs clang-format --dry-run --Werror && \
		printf "${GREEN}${BOLD}[v]${NC} ${GREEN}C++ format OK${NC}\n"; \
	else \
		printf "${YELLOW}${BOLD}[!]${NC} ${YELLOW}clang-format не установлен, пропускаем${NC}\n"; \
	fi

## Локальный запуск C++ тестов (через Docker)
ci-test: ci-test-cpp
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}Все тесты пройдены${NC}\n"

## C++ unit tests через Docker
ci-test-cpp:
	$(require-container)
	@printf "${BLUE}${BOLD}[INFO]${NC} ${CYAN}Запуск C++ unit тестов...${NC}\n"
	@docker exec $(CONTAINER_NAME) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source /root/ws/install/setup.bash && \
		cd /root/ws && \
		colcon test --packages-select quadropted_controller_cpp && \
		colcon test-result --verbose"
	@printf "${GREEN}${BOLD}[v]${NC} ${GREEN}C++ tests OK${NC}\n"
