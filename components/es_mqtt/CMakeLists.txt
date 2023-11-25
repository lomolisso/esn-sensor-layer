idf_component_register(SRCS "es_mqtt.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "edge_sensor" "mqtt" "json"
)
