set(pre_req driver)
idf_component_register(SRCS "input.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
                    