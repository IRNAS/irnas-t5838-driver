# SPDX-License-Identifier: Apache-2.0

if(CONFIG_T5838)
    zephyr_include_directories(.)
    zephyr_library()

    zephyr_library_sources(t5838_nrfx_pdm.c)
endif()