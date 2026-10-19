# Usage: source ./activate_handover.bash
_handover_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /home/ros2/miniconda3/etc/profile.d/conda.sh
conda activate urho
export XDG_CACHE_HOME="${_handover_root}/.cache"
export MPLCONFIGDIR="${XDG_CACHE_HOME}/matplotlib"
export TORCH_EXTENSIONS_DIR="${XDG_CACHE_HOME}/torch_extensions"
export HF_HOME="${XDG_CACHE_HOME}/huggingface"
export CUDA_CACHE_PATH="${XDG_CACHE_HOME}/cuda"
export PIP_CACHE_DIR="${XDG_CACHE_HOME}/pip"
mkdir -p "${MPLCONFIGDIR}" "${TORCH_EXTENSIONS_DIR}" "${HF_HOME}" "${CUDA_CACHE_PATH}" "${PIP_CACHE_DIR}"
export CUDA_HOME="${CONDA_PREFIX}"
export PATH="${CUDA_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${_handover_root}/.zed-sdk/lib:${CUDA_HOME}/lib:${CUDA_HOME}/lib64:${CUDA_HOME}/targets/x86_64-linux/lib:${LD_LIBRARY_PATH:-}"
for _handover_lib in "${CUDA_HOME}"/lib/python3.10/site-packages/nvidia/*/lib; do
    [ ! -d "${_handover_lib}" ] || export LD_LIBRARY_PATH="${_handover_lib}:${LD_LIBRARY_PATH}"
done
export ZED_SDK_ROOT="${_handover_root}/.zed-sdk"
export URHO_ZED_SETTINGS_PATH="${ZED_SDK_ROOT}/settings"
export PYTHONPATH="${_handover_root}/src/SAM2_streaming:${_handover_root}/src/graspnet-baseline/models:${_handover_root}/src/graspnet-baseline/dataset:${_handover_root}/src/graspnet-baseline/utils:${_handover_root}/src/graspnet-baseline/pointnet2:${PYTHONPATH:-}"
if [ -f "${_handover_root}/install/local_setup.bash" ]; then
    source "${_handover_root}/install/local_setup.bash"
fi
unset _handover_root _handover_lib
