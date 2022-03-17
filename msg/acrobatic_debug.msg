uint64 timestamp			# time since system start (microseconds)
bool	   do_acrobatic			# do acrobatic
float32[3] xyz_setpoint			# the position setpoint in the inertial frame
float32[4] quaternion_setpoint		# the quaternion setpoint in the body frame
float32[3] xyz_val			# the position setpoint in the inertial frame
float32[4] quaternion_val		# the quaternion setpoint in the body frame
float32[4] dq_d_real			# the real part of the reference dual quaternion
float32[4] dq_d_dual			# the dual part of the reference dual quaternion
float32[4] dq_v_real			# the real part of the real dual quaternion
float32[4] dq_v_dual			# the dual part of the real dual quaternion
uint64 timestamp_init			# time since system start init(microseconds)
