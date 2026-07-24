#
# Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
#

if(MC_RTC_BUILD_IN_NIX)
  return ()
endif()

find_program(PIP_EXECUTABLE pip3)

if(NOT PIP_EXECUTABLE)
  message(FATAL_ERROR "pip3 not found, cannot install Python package")
endif()

install(CODE "
  execute_process(
    COMMAND \"${PIP_EXECUTABLE}\" install mc_log_ui
    WORKING_DIRECTORY \"${CMAKE_CURRENT_SOURCE_DIR}\"
  )
")
