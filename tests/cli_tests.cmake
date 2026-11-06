# Test the CLI on copies. Leave the examples alone.
file(MAKE_DIRECTORY "${WORK}")
set(input "${WORK}/points.csv")
set(original "x,y\n0,10\n10,0\n")
file(WRITE "${input}" "${original}")
execute_process(COMMAND "${CLI}" "${input}" --output "${input}"
  RESULT_VARIABLE status OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT status EQUAL 1 OR NOT error MATCHES "must differ")
  message(FATAL_ERROR "Same-path output was not rejected: ${status} ${error}")
endif()
file(READ "${input}" after)
if(NOT after STREQUAL original)
  message(FATAL_ERROR "Input changed after rejected export")
endif()
foreach(kind IN ITEMS SYMBOLIC HARD)
  set(alias "${WORK}/alias-${kind}.csv")
  file(REMOVE "${alias}")
  if(kind STREQUAL "SYMBOLIC")
    file(CREATE_LINK "${input}" "${alias}" SYMBOLIC RESULT link_error)
  else()
    file(CREATE_LINK "${input}" "${alias}" RESULT link_error)
  endif()
  if(link_error STREQUAL "0")
    execute_process(COMMAND "${CLI}" "${input}" --output "${alias}"
      RESULT_VARIABLE status OUTPUT_QUIET ERROR_VARIABLE error)
    file(READ "${input}" after)
    if(NOT status EQUAL 1 OR NOT error MATCHES "must differ" OR NOT after STREQUAL original)
      message(FATAL_ERROR "${kind} alias export did not preserve input")
    endif()
  else()
    message(STATUS "${kind} link unavailable; alias check skipped: ${link_error}")
  endif()
endforeach()
execute_process(COMMAND "${CLI}" "${input}" --output "${WORK}/analysis.csv"
  RESULT_VARIABLE status OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Normal export failed: ${error}")
endif()
file(READ "${WORK}/analysis.csv" report)
if(NOT report MATCHES "^distance_m,time_s,")
  message(FATAL_ERROR "Export has no analysis header")
endif()

file(WRITE "${WORK}/slow.csv" "x,y\n0,0\n10,0\n")
execute_process(COMMAND "${CLI}" "${WORK}/slow.csv" --speed 1e-13 --resistance 0 --drag-area 0
  --output "${WORK}/slow-analysis.csv"
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT status EQUAL 0 OR NOT output MATCHES "Elapsed time estimate: 1e\\+14 s")
  message(FATAL_ERROR "Slow positive motion lost its finite travel time: ${status} ${output} ${error}")
endif()
file(READ "${WORK}/slow-analysis.csv" report)
if(NOT report MATCHES "propulsion_work_J,time_resolution_limited" OR
   NOT report MATCHES "10,1e\\+14," OR NOT report MATCHES ",0\n$")
  message(FATAL_ERROR "Slow-motion CSV lost the finite time or its resolution status")
endif()

# Just below a hilltop equilibrium, the sampled travel time is resolution dependent.
file(WRITE "${WORK}/equilibrium.csv" "x,y\n0,0\n1,1\n2,0\n")
execute_process(COMMAND "${CLI}" "${WORK}/equilibrium.csv" --speed 4.4286905513922665
  --resistance 0 --drag-area 0 --output "${WORK}/equilibrium-analysis.csv"
  RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT status EQUAL 2 OR NOT output MATCHES "Elapsed time: unresolved" OR
   output MATCHES "Elapsed time estimate:")
  message(FATAL_ERROR "CLI concealed an unresolved travel time: ${status} ${output} ${error}")
endif()
file(READ "${WORK}/equilibrium-analysis.csv" report)
if(NOT report MATCHES "time_resolution_limited" OR NOT report MATCHES ",1\n$")
  message(FATAL_ERROR "Unresolved travel time was not flagged in the CSV")
endif()
