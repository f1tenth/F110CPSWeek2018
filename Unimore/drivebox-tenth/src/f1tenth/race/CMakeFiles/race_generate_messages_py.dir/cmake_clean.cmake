FILE(REMOVE_RECURSE
  "CMakeFiles/race_generate_messages_py"
  "devel/lib/python2.7/dist-packages/race/msg/_drive_param.py"
  "devel/lib/python2.7/dist-packages/race/msg/_pid_input.py"
  "devel/lib/python2.7/dist-packages/race/msg/_drive_values.py"
  "devel/lib/python2.7/dist-packages/race/msg/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/race_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
