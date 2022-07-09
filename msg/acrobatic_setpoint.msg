uint64 timestamp			# time since system start (microseconds)
bool	   setpoint_generated		# the setpoint received
float32[4] quat_d	                # pose real
float32[3] xyz_d	                # pose dual
float32[8] ref_twist			# reference twist
bool       acrobatic_finish		# acrobatic finished
float32[3] xyz_first_acro		# xyz first acro
