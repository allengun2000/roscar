file(REMOVE_RECURSE
  "interface_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/interface_gennodejs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
