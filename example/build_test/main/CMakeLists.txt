set(srcs            "main.c"
                    "OD.c")

set(include_dirs    "."
                    "../../../CANopenNode"
                    "../../../port")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${include_dirs})
