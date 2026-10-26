if(CONFIG_DUST_BUF_BIPBUF)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/buffer/bipbuf)
endif()

if(CONFIG_DUST_BUF_RINGBUF)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/buffer/ringbuf)
endif()

if(CONFIG_DUST_CTL_EXECTIMER)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/controller/timer)
endif()

if(CONFIG_DUST_CTL_PID)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/controller/pid/pid.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/controller/pid)
endif()

if(CONFIG_DUST_CTL_TIMER)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/controller/timer)
endif()

if(CONFIG_DUST_FLT_HPF)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/hpf)
endif()

if(CONFIG_DUST_FLT_KALMAN)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/kalman/kalman_check.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/kalman)
endif()

if(CONFIG_DUST_FLT_KALMAN_EKF)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/kalman/kalman_check.cpp)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/kalman)
endif()

if(CONFIG_DUST_FLT_LPF)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/lpf)
endif()

if(CONFIG_DUST_FLT_QUATERNION)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/filter/quaternion/quaternion.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/filter/quaternion
		${CMAKE_CURRENT_SOURCE_DIR}/filter/kalman
	)
endif()

if(CONFIG_DUST_ID_RLS)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/identify/rls)
endif()

if(CONFIG_DUST_ID_MOTOR_PLANT)
	target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/identify/motor/motorplant.cpp)
	target_include_directories(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/identify/motor
		${CMAKE_CURRENT_SOURCE_DIR}/identify/rls
	)
endif()

if(CONFIG_DUST_ID_STABILITY)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/identify)
endif()

if(CONFIG_DUST_MATH_EIGEN)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/math/eigen)
endif()

if(CONFIG_DUST_MOD_CTL_POWER)
	target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/controller/power_ctrl)
endif()
