# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/tux/esp/v5.1.2/esp-idf/components/bootloader/subproject"
  "/home/tux/projects/tautuk_esp32s3_core/build/bootloader"
  "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix"
  "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix/tmp"
  "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix/src/bootloader-stamp"
  "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix/src"
  "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/tux/projects/tautuk_esp32s3_core/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
