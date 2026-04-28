target_sources(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/tb45_cellular.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_cell_input_bridge.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tb45_sms.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_sms_at_helper.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
