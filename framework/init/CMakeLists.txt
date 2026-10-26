target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Irq_handlers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Init_entry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
)

zephyr_linker_sources(ROM_SECTIONS ../cmd/linker/dust_init.ld)
