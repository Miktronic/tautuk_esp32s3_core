set(COMPONENT_SRCS
	"app_main.c"
    "MQLibrary.c"
    "aws_iot_mqtt.c"
	"app_ble.c"
	)

set(COMPONENT_ADD_INCLUDEDIRS
	"."
	"${CMAKE_CURRENT_LIST_DIR}"
	)

idf_component_register(SRCS "${COMPONENT_SRCS}"
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
   )
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
