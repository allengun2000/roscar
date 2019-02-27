file(REMOVE_RECURSE
  "connect_test_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/doxygen.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
