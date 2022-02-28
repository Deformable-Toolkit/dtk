find_package(CGAL)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

set(Boost_INCLUDE_DIR D:\\boost\\boost_1_71_0)
set(Boost_LIBRARIES_DIR D:\\boost\\boost_1_71_0\\lib64-msvc-14.2)
set(GLM_INCLUDE_DIR D:\\glm-0.9.9.8\\glm)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARIES_DIR})
include_directories(${GLM_INCLUDE_DIR}) 

#link
add_library(dtk STATIC ${dir_source})


target_link_libraries(dtk Boost)
target_link_libraries(dtk CGAL::CGAL)

install(
    TARGETS dtk
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

#install
install(TARGETS dtk DESTINATION ${DTK_INSTALL_LIBDIR})
install(FILES ${dir_headers} DESTINATION ${DTK_INSTALL_INCLUDEDIR})