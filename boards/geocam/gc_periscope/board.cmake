# Board-specific CMake configuration if needed
board_runner_args(openocd --target-handle=_CHIPNAME.core0)
board_runner_args(jlink --device=RP2040_M0_0)
board_runner_args(uf2 "--board-id=RPI-RP2")

include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)