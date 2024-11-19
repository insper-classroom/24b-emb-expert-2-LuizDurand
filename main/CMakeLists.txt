add_executable(main
        main.c
)


target_link_libraries(main pico_stdlib freertos oled1_lib hardware_dma)
pico_add_extra_outputs(main)
