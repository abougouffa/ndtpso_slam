.PHONY: clean All

All:
	@echo "----------Building project:[ ndtpso_slam - Debug ]----------"
	@"$(MAKE)" -f  "ndtpso_slam.mk"
clean:
	@echo "----------Cleaning project:[ ndtpso_slam - Debug ]----------"
	@"$(MAKE)" -f  "ndtpso_slam.mk" clean
