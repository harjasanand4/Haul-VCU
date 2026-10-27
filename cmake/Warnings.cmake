function(haul_set_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
    if(HAUL_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    # -Wconversion matters in control code: a silent double -> int16 truncation is a bug.
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
      -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Woverloaded-virtual)
    if(HAUL_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()

  if(HAUL_SANITIZE)
    target_compile_options(${target} PRIVATE -fsanitize=${HAUL_SANITIZE} -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=${HAUL_SANITIZE})
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      # GCC's flow analysis reports false positives inside libstdc++ (std::regex, std::function)
      # once sanitizer instrumentation is added. Release builds keep the warning.
      target_compile_options(${target} PRIVATE -Wno-maybe-uninitialized)
    endif()
    if(HAUL_SANITIZE MATCHES "undefined")
      # Make UBSan findings fatal, so a test run fails instead of printing and carrying on.
      target_compile_options(${target} PRIVATE -fno-sanitize-recover=undefined)
    endif()
  endif()
endfunction()
