target_sources(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/tb45_cellular.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_async_job.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_async_dispatcher.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_delayable_retry.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_ping.c
    ${CMAKE_CURRENT_LIST_DIR}/tb45_sms_at_helper.c
)

if(CONFIG_APP_TB45_SMS_ENABLE)
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/tb45_sms.c
        ${CMAKE_CURRENT_LIST_DIR}/tb45_sms_event.c
    )
endif()

if(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/tb45_ppp_probe.c
    )
endif()

if(CONFIG_SHELL AND CONFIG_SHELL_BACKEND_SERIAL AND CONFIG_SHELL_LOG_BACKEND AND CONFIG_UART_CONSOLE)
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/tb45_shell.c
    )
endif()

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)
