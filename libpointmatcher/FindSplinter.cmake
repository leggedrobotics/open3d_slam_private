# https://github.com/bgrimstad/splinter
#
# Splinter_ROOT root search path
# Splinter_INCLUDE_DIR inlcude directories
# Splinter_LIBRARIES libraries
#


find_path(Splinter_INCLUDE_DIR NAMES data_table.h PATHS /usr/local/include/SPLINTER)

find_library(Splinter_LIBRARIES NAMES splinter-4-0 PATHS /usr/local/lib /usr/local/lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Splinter DEFAULT_MSG
                                  Splinter_INCLUDE_DIR Splinter_LIBRARIES)
mark_as_advanced(Splinter_INCLUDE_DIR Splinter_LIBRARIES)
