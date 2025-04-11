idf_component_register(SRCS "speedController.cpp" "main.cpp" "ballHandler/ballHandler.cpp"
                    INCLUDE_DIRS "." "include"
                    REQUIRES embed-ros qmd)
