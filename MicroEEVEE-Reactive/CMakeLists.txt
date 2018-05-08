cmake_minimum_required(VERSION 3.7)
project(MicroEEVEE_Reactive)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        bluetooth_comm.c
        bluetooth_comm.h
        DodgeObstacle.c
        DodgeObstacle.h
        FollowTheBeacon.c
        FollowTheBeacon.h
        FollowTheWall.c
        FollowTheWall.h
        main.c
        rmi-mr32.c
        rmi-mr32.h
        Sensors.c
        Sensors.h
        StopAtBeacon.c
        StopAtBeacon.h
        Wander.c
        Wander.h)

add_executable(MicroEEVEE_Reactive ${SOURCE_FILES})