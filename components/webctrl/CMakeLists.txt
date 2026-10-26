# The ESP32-P4 has no Wi-Fi of its own (see the transport choice in main/Kconfig):
# no web control there, and no Wi-Fi requirements.
idf_build_get_property(webctrl_idf_target IDF_TARGET)
if(webctrl_idf_target STREQUAL "esp32p4")
    idf_component_register(INCLUDE_DIRS "include")
    return()
endif()

# Requirements must not depend on Kconfig values (they are resolved before the
# configuration is known), so only the sources are conditional.
set(srcs "")
if(CONFIG_WEBCTRL_ENABLE)
    set(srcs "webctrl.c" "webctrl_core.c" "dns_server.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES crazyflie platform wifi config
                                     esp_http_server esp_netif esp_wifi esp_event
                                     esp_timer esp_app_format lwip)

if(CONFIG_WEBCTRL_ENABLE)
    # The control page is compressed at build time and linked into the
    # firmware (symbols _binary_index_html_gz_start / _end).
    idf_build_get_property(python PYTHON)
    set(WEB_PAGE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html")
    set(WEB_PAGE_GZ "${CMAKE_CURRENT_BINARY_DIR}/index.html.gz")
    add_custom_command(OUTPUT ${WEB_PAGE_GZ}
                       COMMAND ${python} "${CMAKE_CURRENT_SOURCE_DIR}/tools/gzip_file.py"
                               ${WEB_PAGE_SRC} ${WEB_PAGE_GZ}
                       DEPENDS ${WEB_PAGE_SRC} "${CMAKE_CURRENT_SOURCE_DIR}/tools/gzip_file.py"
                       VERBATIM)
    add_custom_target(webctrl_page DEPENDS ${WEB_PAGE_GZ})
    add_dependencies(${COMPONENT_LIB} webctrl_page)
    target_add_binary_data(${COMPONENT_LIB} ${WEB_PAGE_GZ} BINARY)
    set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
                 ADDITIONAL_CLEAN_FILES ${WEB_PAGE_GZ})
endif()
