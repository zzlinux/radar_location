cmake_minimum_required(VERSION 3.6)
project(radar_location)

set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
find_package(OpenCV 3.1.0 REQUIRED)
find_package(Boost  REQUIRED)
include_directories(radar
                    include
                    {OPENCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
        )

set(urg radar/Urg_driver.cpp
        radar/urg_connection.c
        radar/ticks.cpp
        radar/urg_utils.c
        radar/urg_debug.c
        radar/urg_ring_buffer.c
        radar/urg_sensor.c
        radar/urg_serial.c
        radar/urg_tcpclient.c
        radar/Connection_information.cpp
        radar/urg_serial_utils_linux.c)
set(boost
        /usr/lib/x86_64-linux-gnu/libboost_atomic.so
        /usr/lib/x86_64-linux-gnu/libboost_chrono.so
        /usr/lib/x86_64-linux-gnu/libboost_date_time.so
        /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
        /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
        /usr/lib/x86_64-linux-gnu/libboost_regex.so
        /usr/lib/x86_64-linux-gnu/libboost_serialization.so
        /usr/lib/x86_64-linux-gnu/libboost_system.so
        /usr/lib/x86_64-linux-gnu/libboost_thread.so
        /usr/lib/x86_64-linux-gnu/libboost_wserialization.so
        )
set(SOURCE_FILES src/main.cpp
        ${urg}
        src/Radar.cpp
        src/Base.cpp
        src/serialbase.cpp
        src/serialapp.cpp
        src/utility.cpp
        src/Algorithm.cpp)
add_executable(main ${SOURCE_FILES})
target_link_libraries(main ${OpenCV_LIBS}
        ${boost}
        ${BOOST_LIBS})
