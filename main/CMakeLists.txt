idf_component_register(SRCS "hello_world_main.c" 
                            "rmt_capture.c" 
                            "mcpwm_control.c"
                    INCLUDE_DIRS ".")

# Opcjonalnie: wyłączenie ostrzeżeń o formacie
add_compile_options(-Wno-format)
