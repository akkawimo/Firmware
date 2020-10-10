px4_add_module(
	MODULE modules__integrated_accel
	MAIN integrated_accel
	COMPILE_FLAGS
	SRCS
		integrated_accel.cpp
	DEPENDS
		modules__uORB
		px4_work_queue
	)
