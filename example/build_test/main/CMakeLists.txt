set(co_dir "../../../CANopenNode")
set(co_port_dir "../../../port")
set(co_files    "${co_dir}/CANopen.c"
                "${co_dir}/301/CO_Emergency.c"
                "${co_dir}/301/CO_fifo.c"
                "${co_dir}/301/CO_HBconsumer.c"
                "${co_dir}/301/CO_NMT_Heartbeat.c"
                "${co_dir}/301/CO_ODinterface.c"
                "${co_dir}/301/CO_PDO.c"
                "${co_dir}/301/CO_SDOserver.c"
                "${co_dir}/301/CO_SYNC.c"
                "${co_dir}/301/CO_TIME.c"
                "${co_dir}/305/CO_LSSmaster.c"
                "${co_dir}/305/CO_LSSslave.c")

set(srcs            "main.c"
                    "OD.c"
                    ${co_files})

set(include_dirs    "."
                    ${co_dir}
                    ${co_port_dir})

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${include_dirs})
