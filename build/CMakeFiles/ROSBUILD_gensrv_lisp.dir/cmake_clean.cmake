FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/quadrotorTestControl/msg"
  "../src/quadrotorTestControl/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/getTargetVelocity.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getTargetVelocity.lisp"
  "../srv_gen/lisp/getTargetPosition.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getTargetPosition.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
