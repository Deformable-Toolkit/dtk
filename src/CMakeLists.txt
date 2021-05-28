find_package(CGAL)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

#link
add_library(dtk ${dir_source})
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