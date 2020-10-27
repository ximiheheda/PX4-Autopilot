#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default gazebo'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris

function cleanup() {
	pkill -x px4
	pkill gazebo
	pkill gzclient
	pkill gzserver
}

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number
	WORLD_FILE=$3
	HITL_MODEL_NAME="temp_${MODEL}_hitl"
	SITL_MODEL_NAME="${MODEL}_${N}"

	SUPPORTED_MODELS=("iris" "iris_rtps" "plane" "standard_vtol")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL "* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi
	sitl_path=${SCRIPT_DIR}/sitl_gazebo
	jinja_script=${sitl_path}/scripts/jinja_gen.py
	jinja_model=${sitl_path}/models/${MODEL}/${MODEL}.sdf.jinja
	
    if [ $hitl == true ]; then
		mkdir -p ${sitl_path}/models/${HITL_MODEL_NAME}
		serial_enabled="--serial_enabled 1"
		hil_mode="--hil_mode 1"
		mavlink_tcp="--mavlink_tcp_port 4560"
		mavlink_udp="--mavlink_udp_port 14560"
		enable_lockstep="--enable_lockstep 0"
		output_file="--output-file ${sitl_path}/models/${HITL_MODEL_NAME}/${HITL_MODEL_NAME}.sdf"
		jinja_world=${sitl_path}/worlds/${WORLD_FILE}.world.jinja
		output_world="--output-file ${sitl_path}/worlds/temp_${WORLD_FILE}_hitl.world"
		model_name="--model_name ${HITL_MODEL_NAME}"
		jinja_config=${sitl_path}/scripts/model.config.jinja
		output_config="--output-file ${sitl_path}/models/${HITL_MODEL_NAME}/model.config"

		python3 $jinja_script $jinja_model $sitl_path $model_name $enable_lockstep $mavlink_tcp $mavlink_udp $serial_enabled $serial_device $serial_baudrate $hil_mode $output_file
		python3 $jinja_script $jinja_world $sitl_path $model_name $output_world
		python3 $jinja_script $jinja_config $sitl_path $model_name $output_config
		sleep 1
		source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}
		sleep 2
		gazebo ${sitl_path}/worlds/temp_${WORLD_FILE}_hitl.world --verbose

	else
		mavlink_tcp="--mavlink_tcp_port $((4560+${N}))"
		mavlink_udp="--mavlink_udp_port $((14560+${N}))"
		serial_enabled="--serial_enabled 0"
    	serial_device="--serial_device /dev/ttyACM0"
    	serial_baudrate="--serial_baudrate 921600"
    	enable_lockstep="--enable_lockstep 1"
    	hil_mode="--hil_mode 0"
    	model_name="--model_name ${SITL_MODEL_NAME}"
    	output_file="--output-file /tmp/${SITL_MODEL_NAME}.sdf"
		working_dir="$build_path/instance_$n"
		[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
		pushd "$working_dir" &>/dev/null
		echo "starting instance $N in $(pwd)"
		../bin/px4 -i $N -d "$build_path/etc" -w sitl_${SITL_MODEL_NAME} -s etc/init.d-posix/rcS >out.log 2>err.log &
		python3 $jinja_script $jinja_model $sitl_path $enable_lockstep $model_name $mavlink_tcp $mavlink_udp $serial_enabled $serial_device $serial_baudrate $hil_mode $output_file
		echo "Spawning ${MODEL}_${N}"
		gz model --spawn-file=/tmp/${SITL_MODEL_NAME}.sdf --model-name=${SITL_MODEL_NAME} -x 0.0 -y $((3*${N})) -z 0.0	
		popd &>/dev/null
	fi
	
}

if [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-h <run_hitl>] [-w <world>] [-s <script>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	echo "-h flag is used to launch a single vehicle in HITL mode"
	exit 1
fi

while getopts n:m:h:w:s:t: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		h) HITL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
hitl=${HITL:=false}
if [ "$hitl" == "True" ] || [ "$hitl" == "1" ] || [ "$hitl" == "true" ];then
	hitl=true
else
	hitl=false
fi
target=${TARGET:=px4_sitl_default}
export PX4_SIM_MODEL=${VEHICLE_MODEL:=iris}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

if [ $hitl == true ]; then
	echo "HITL mode is currently turned on, disabling multiple vehicle spawn and script spawn."
	spawn_model ${PX4_SIM_MODEL} 0 ${world}

else
	echo ${SCRIPT}

	build_path=${src_path}/build/${target}
	mavlink_udp_port=14560
	mavlink_tcp_port=4560

	echo "killing running instances"
	pkill -x px4 || true

	sleep 1

	source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}
	echo "HITL mode is currently turned off."
	echo "Starting gazebo"
	gzserver ${src_path}/Tools/sitl_gazebo/worlds/${world}.world --verbose &
	sleep 5

	n=0
	if [ -z ${SCRIPT} ]; then
		if [ $num_vehicles -gt 255 ]; then
			echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi
		while [ $n -lt $num_vehicles ]; do
			spawn_model ${PX4_SIM_MODEL} $n ${world}
			n=$(($n + 1))
		done
	else
		IFS=,
		for target in ${SCRIPT}; do
			target="$(echo "$target" | tr -d ' ')" #Remove spaces
			target_vehicle="${target%:*}"
			target_number="${target#*:}"

			if [ $n -gt 255 ]; then
				echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
				exit 1
			fi

			m=0
			while [ $m -lt ${target_number} ]; do
				spawn_model ${target_vehicle} $n ${world}
				m=$(($m + 1))
				n=$(($n + 1))
			done
		done

	fi
	echo "Starting gazebo client"
	gzclient
fi

trap "cleanup" SIGINT SIGTERM EXIT
