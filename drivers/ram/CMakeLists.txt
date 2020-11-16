# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_USERSPACE ram_handlers.c)
zephyr_library_sources_ifdef(CONFIG_EEPROM_SHELL ram_shell.c)

zephyr_library_sources_ifdef(CONFIG_RAM_MCP23x ram_mcp23x.c)
zephyr_library_sources_ifdef(CONFIG_RAM_SIMULATOR ram_simulator.c)
