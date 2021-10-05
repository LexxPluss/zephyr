zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADIS16470 adis16470.c)
