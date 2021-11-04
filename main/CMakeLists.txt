idf_component_register(
    SRCS "main.c" "ssd1306.c" "i2cdev.c"
    INCLUDE_DIRS ""
)
#EXCLUDE_COMPONENTS := max7219 mcp23x17    $(HOME)/esp/esp-idf-lib/components/i2cdev    $(HOME)/esp/esp-idf-lib/components/esp_idf_lib_helpers
