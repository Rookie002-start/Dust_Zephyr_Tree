# USB 设备栈装配：中间层（栈内核） + 芯片 HAL + Stream 适配层
#
# ⚠️ 本目录由**应用** CMakeLists（add_subdirectory(${FW_ROOT}/drivers)）装配，属于 Zephyr 的
#    "application mode"：此时 zephyr_library() 创建的库**不会进链接**
#    （extensions.cmake 只在配置阶段给一条警告）。所以这里与 framework 其它层保持
#    一致，统一用 target_sources(app PRIVATE ...) 直接编进 app。

# ---- 中间层（栈内核）----
if(CONFIG_DUST_USB_DEVICE_STACK)
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/usb_dev_port.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/usb_cdc_acm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/usb_descriptor.cpp
    )
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/interface
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        # 接收环形缓冲用 DUST_BUF_BIPBUF（algorithm 层默认只把 include 加给 app 目标）
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../algorithm/buffer/bipbuf
    )
    # 全局暴露，供业务代码裸文件名引用
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/interface
                               ${CMAKE_CURRENT_SOURCE_DIR}/core)
endif()

# ---- 芯片 HAL（HPM）----
if(CONFIG_DUST_USB_DEVICE_HAL_HPM)
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/hal/hpm/usb_hal_hpm.cpp
        ${SDK_GLUE_DIR}/../sdk_env/hpm_sdk/components/usb/device/hpm_usb_device.c
        ${SDK_GLUE_DIR}/../sdk_env/hpm_sdk/drivers/src/hpm_usb_drv.c
    )
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/hal/hpm
        ${SDK_GLUE_DIR}/../sdk_env/hpm_sdk/components/usb/device
        ${SDK_GLUE_DIR}/../sdk_env/hpm_sdk/drivers/inc
    )
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hal/hpm)
endif()

# ---- Stream 适配层（应用使能 DUST_COM_USB 时编译）----
if(CONFIG_DUST_COM_USB)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/usb.cpp)
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..     # 提供 stream.hpp
    )
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
endif()
