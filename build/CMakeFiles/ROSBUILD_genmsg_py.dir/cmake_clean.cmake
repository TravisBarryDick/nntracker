FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/NNTracker/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/NNTracker/msg/__init__.py"
  "../src/NNTracker/msg/_NNTrackerROI.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
