ADD_EXECUTABLE(test_queries_2D test_queries_2D.cpp)
TARGET_LINK_LIBRARIES(test_queries_2D
    ${Boost_LIBRARIES}
    )
ADD_TEST(queries_2D test_queries_2D)

ADD_EXECUTABLE(test_queries_3D test_queries_3D.cpp)
TARGET_LINK_LIBRARIES(test_queries_3D
    ${Boost_LIBRARIES}
    )
ADD_TEST(queries_3D test_queries_3D)

ADD_EXECUTABLE(test_queries_ND test_queries_ND.cpp)
TARGET_LINK_LIBRARIES(test_queries_ND
    ${Boost_LIBRARIES}
    )
ADD_TEST(queries_ND test_queries_ND)
