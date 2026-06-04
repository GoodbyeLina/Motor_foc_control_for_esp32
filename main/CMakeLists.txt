idf_component_register(SRCS "sensorless_startup.c" 
                            "smo_observer.c" 
                            "motor_test.c" 
                            "motor_control.c" 
                            "serial_cmd.c" 
                            "as5600.c"
                            "current_sense.c"
                            "foc_core.c"
                            "ledc_pwm.c"
                            "main.c"
                            "pid.c"
                       INCLUDE_DIRS ".")

