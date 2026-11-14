cmake_minimum_required(VERSION 3.10)
project(pathfinder_bot_plugin C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

set(HOST_APP_DIR "" CACHE PATH "Path to the FrameTee project root")
if(NOT HOST_APP_DIR)
    message(FATAL_ERROR "Please provide the path to the host application root via -DHOST_APP_DIR=../..")
endif()

add_library(${PROJECT_NAME} SHARED pathfinder_bot.c)

set_source_files_properties(pathfinder_bot.c PROPERTIES COMPILE_DEFINITIONS CIMGUI_DEFINE_ENUMS_AND_STRUCTS)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${HOST_APP_DIR}/include"
    "${HOST_APP_DIR}/src/plugins"
    "${HOST_APP_DIR}/libs/cglm/include"
    "${HOST_APP_DIR}/libs/cimgui"
    "${HOST_APP_DIR}/libs/cimgui/imgui"
)

# With the DDNet physics available (the top-level build defines the target, as for grenade_lineups)
# the plugin reads the map: it finds the finish tiles and steers the search with a distance field.
# That variant is bound to the "ddnet" game (official DDNet physics). Without it the plugin is global
# and needs a manual goal.
if(TARGET ddnet_physics)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PF_WITH_DDNET=1)
    target_include_directories(${PROJECT_NAME} PRIVATE "${HOST_APP_DIR}/games/ddnet/include")
    target_link_libraries(${PROJECT_NAME} PRIVATE ddnet_physics)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-ffp-contract=off>)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-Wall>
    $<$<C_COMPILER_ID:MSVC>:/W4>
)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME pathfinder_bot)

if(WIN32)
    if(TARGET frametee)
        target_link_libraries(${PROJECT_NAME} PRIVATE frametee)
    else()
        target_link_libraries(${PROJECT_NAME} PRIVATE
            "${HOST_APP_DIR}/build/$<IF:$<BOOL:${CMAKE_CONFIGURATION_TYPES}>,$<CONFIG>,${CMAKE_BUILD_TYPE}>/frametee.lib")
    endif()
endif()
