cmake_minimum_required(VERSION 3.7)
project(MicroEEVEE)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        AStar.c
        AStar.h
        bluetooth_comm.c
        bluetooth_comm.h
        DodgeObstacle.c
        DodgeObstacle.h
        FollowTheBeacon.c
        FollowTheBeacon.h
        FollowTheWall.c
        FollowTheWall.h
        main_18_v0.c
        MazeMap.c
        MazeMap.h
        mr32.c
        mr32.h
        Sensors.c
        Sensors.h
        StopAtBeacon.c
        StopAtBeacon.h
        Wander.c
        Wander.h)

add_executable(MicroEEVEE ${SOURCE_FILES})