
COMPILER = gcc
CFLAGS = -Wall
LDFLAGS = -lm

BUILD_DIR = build

HEADER  = ./include/A_define_IP_parameter.c
HEADER += ./include/A_define_IP_parameter.h

HEADER += ./include/B_calc_Optimal_FB.c
HEADER += ./include/B_calc_Optimal_FB.h

HEADER += ./include/C_simulation_loop_function.c
HEADER += ./include/C_simulation_loop_function.h

HEADER += ./include/D_time_sampling.c
HEADER += ./include/D_time_sampling.h

HEADER += ./include/E_IP_pos_animation.c
HEADER += ./include/E_IP_pos_animation.h

HEADER += ./include/F_arduino_acceleration.c
HEADER += ./include/F_arduino_acceleration.h

.PHONY: all clean

all: $(BUILD_DIR) $(BUILD_DIR)/test 

$(BUILD_DIR)/test: $(BUILD_DIR)/test.o
	$(COMPILER) $(CFLAGS) -o $@ $^ $(HEADER) $(LDFLAGS) -lwiringPi

$(BUILD_DIR)/test.o: test.c
	$(COMPILER) $(CFLAGS) -o $@ -c $<

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

clean: $(BUILD_DIR)
	rm build/*