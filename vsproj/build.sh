echo "========  build.sh =========="
CURR_PATH=$(pwd)

echo "== 1 =="
#cd /home/av/imx/
#pwd
#MACHINE=ucm-imx8m-plus
#source compulab-setup-env -b build-${MACHINE}

cd /home/av/compulab-nxp-bsp/
pwd
export MACHINE=ucm-imx8m-plus
source setup-environment build-${MACHINE}

echo "== 2 =="
cd ${CURR_PATH}
pwd

echo "== 3 =="
bitbake -c $1 tflow-capture

echo "== 4 =="
cat /home/av/compulab-nxp-bsp/build-ucm-imx8m-plus/workspace/sources/tflow-capture/oe-logs/log.do_compile >&2

echo "========  EOF build.sh =========="