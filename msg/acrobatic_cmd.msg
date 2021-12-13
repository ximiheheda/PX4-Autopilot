uint64 timestamp			# time since system start (microseconds)
float32[3] body_rates_cmd               # body rates command in acrobatic
float32[4] quaternion_cmd               # the quaternion_cmd readed from the file
float32[3] angular_velocity             # the body rates in acrobatic
bool acrobatic_finish                   # the finish tag of the acrobatic
float32[3] control_input_pqr            # the control input p,q,r
float32    control_input_th             # the control input throttle
float32    alt_sp_acrobatic             # altitude sepoint acrobatic
float32[3] euler_cmd                    # euler angles setpoint acrobatic
