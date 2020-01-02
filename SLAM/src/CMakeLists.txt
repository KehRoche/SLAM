add_library( ownslam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    vo.cpp
)


target_link_libraries( ownslam
    ${THIRD_PARTY_LIBS}
)
