PI := openclaw@192.168.1.108

deploy: ## Deploy all (hmi + ros2)
	./scripts/deploy.sh all

deploy-hmi: ## Deploy HMI only
	./scripts/deploy.sh hmi

deploy-ros2: ## Deploy ROS2 packages + rebuild
	./scripts/deploy.sh ros2

deploy-system: ## Deploy systemd units (requires sudo on Pi)
	./scripts/deploy.sh system

sync-bags: ## Pull bags from Pi → local bags/
	rsync -avz --progress $(PI):~/bags/ bags/

bootstrap: ## Full fresh install on Pi
	ssh $(PI) "bash -s" < scripts/bootstrap.sh

ota: ## Trigger OTA update on Pi via SSH
	ssh $(PI) "~/scripts/update.sh"

flash-rp2040: ## Build + flash spin_controller firmware
	cd firmware/spin_controller && pio run -t upload

flash-cyd: ## Build + flash CYD HMI firmware
	cd firmware/cyd_hmi && pio run -t upload

status: ## Check Pi HMI health
	@curl -sf http://192.168.1.108:3000/api/status | python3 -m json.tool || echo "Pi unreachable"

logs-hmi: ## Tail HMI log on Pi
	ssh $(PI) "sudo journalctl -u hmi -f"

logs-ros2: ## Tail ROS2 bridge log on Pi
	ssh $(PI) "sudo journalctl -u hmi_bridge -f"

.PHONY: deploy deploy-hmi deploy-ros2 deploy-system sync-bags bootstrap ota \
        flash-rp2040 flash-cyd status logs-hmi logs-ros2
