FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/quadrotorTestControl/msg"
  "../src/quadrotorTestControl/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/quadrotorTestControl/srv/__init__.py"
  "../src/quadrotorTestControl/srv/_getTargetVelocity.py"
  "../src/quadrotorTestControl/srv/_getTargetPosition.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
