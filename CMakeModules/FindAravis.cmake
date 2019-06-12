INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(Aravis_INCLUDE_DIRS arv.h
  "$ENV{ARAVIS_INCLUDE_PATH}"
  /usr/local/include/aravis-0.6
)

FIND_LIBRARY(Aravis_LIBRARIES aravis-0.6
  "$ENV{ARAVIS_LIBRARY}"
  /usr/local/lib
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(ARAVIS DEFAULT_MSG
  Aravis_INCLUDE_DIRS
  Aravis_LIBRARIES)

