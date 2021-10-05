DATE=`date +%d-%m-%y_%H-%M`
NAME=${PWD##*/}
echo  "rm /upb/users/c/clienen/profiles/unix/imt/scratch/hw_build_${NAME}/ -r -f"
ssh clienen@cc-9.cs.upb.de "rm /upb/users/c/clienen/profiles/unix/imt/scratch/hw_build_${NAME}/ -r -f"
rsync -r -v build.cfg build.msg src --exclude 'src/application' --exclude 'src/_build' clienen@cc-9.cs.upb.de:/upb/users/c/clienen/profiles/unix/imt/scratch/hw_build_${NAME}
sshcommand="source /opt/ros/dashing/setup.bash; source /opt/Xilinx/Vivado/2017.1/settings64.sh; source bashinit; export XILINXD_LICENSE_FILE=27000@license5.uni-paderborn.de; source /upb/users/c/clienen/profiles/unix/imt/scratch/ReconROS_MASTER/tools/settings.sh; cd /upb/users/c/clienen/profiles/unix/imt/scratch/hw_build_${NAME}/; rdk export_hw | tee rdk_export_hw.log; rdk build_hw | tee rdk_build_hw.log; exec bash"
ssh -t clienen@cc-9.cs.upb.de "screen bash -c '" $sshcommand "'"
rsync -r -v clienen@cc-9.cs.upb.de:/upb/users/c/clienen/profiles/unix/imt/scratch/hw_build_${NAME} hw_build_remote_${DATE}
