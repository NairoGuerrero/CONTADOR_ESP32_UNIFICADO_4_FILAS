idf_component_register(
    SRCS
        "main.c"
        "config.c"
        "ds3231.c"
        "gpio_hw.c"
        "sensores.c"
        "uart_comm.c"
        "diagnostico.c"
    INCLUDE_DIRS "."
    EMBED_FILES "diag_page.html"
)