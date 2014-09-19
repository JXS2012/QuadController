FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/quadrotorTestControl/msg"
  "../src/quadrotorTestControl/srv"
  "CMakeFiles/rospack_gensrv_all"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_gensrv_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
