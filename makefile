SHELL:=/bin/bash
WORKSPACE=$(shell pwd)

RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[1;33m
NC := \033[0m

AUTO_START?=false
CONFIG?=$(WORKSPACE)/config/carla_default_interface_config.yaml
GPUS?=all
export GPUS
DOCKER_COMPOSE ?= $(shell if docker compose version >/dev/null 2>&1; then echo "docker compose"; else echo "docker-compose"; fi)

ifneq (,$(wildcard /.dockerenv))
CARLA_AS_USER := sudo -u developer
else
CARLA_AS_USER :=
endif
download_carla_assets:
	@bash scripts/download_assests/download_carla_asurt.sh
launch_designer: kill_stale_sim
	@$(CARLA_AS_USER) setsid bash ASU_RT_Carla/CarlaUE4.sh -vulkan -renderoffscreen & \
	C_PID=$$!; \
	cleanup() { \
		kill -9 -$$C_PID >/dev/null 2>&1 || true; \
		pkill -9 -f '[C]arlaUE4' >/dev/null 2>&1 || true; \
		for i in $$(seq 1 20); do nc -z localhost 2000 >/dev/null 2>&1 || break; sleep 1; done; \
	}; \
	trap 'cleanup; exit' INT TERM; \
	echo -e "${YELLOW}Waiting for CARLA server on :2000...${NC}"; \
	until nc -z localhost 2000; do sleep 1; done; \
	sleep 3; \
	echo -e "${GREEN}CARLA server up. Launching Scenario Designer...${NC}"; \
	cd $(WORKSPACE)/scenario_runner && python3 -m tools.scenario_designer.main --host localhost --port 2000; \
	cleanup
launch_designer_no_server:
	echo -e "${GREEN}Launching Scenario Designer...${NC}"; \
	cd $(WORKSPACE)/scenario_runner && python3 -m tools.scenario_designer.main --host localhost --port 2000
# Same output as scenario_runner.py --list, minus its runtime imports (shapely etc.).
list_scenarios:
	@cd $(WORKSPACE)/scenario_runner && python3 -c "from srunner.tools.scenario_parser import ScenarioConfigurationParser as P; \
	print(*P.get_list_of_scenarios(''), sep='\n')"
run_csv_scenarios_standalone:
	@source install/ros_apps/setup.bash && \
	PYTHONPATH=$(CARLA_PYAPI):$$PYTHONPATH python3 scenario_runner/run_csv_scenarios.py --csv $(CSV) --host localhost --port 2000 \
		--carla-config $(abspath $(CONFIG)) --sm-stack none
