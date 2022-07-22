declare -a ROS_VERSIONS=( "foxy" "galactic" "humble" "rolling" )

ORGANIZATION="deepmind"
MODEL_NAME="vision-perceiver"
declare -a MODEL_VERSIONS=( "conv" "learned" "fourier" )

for VERSION in "${ROS_VERSIONS[@]}"
do
  for MODEL_VERSION in "${MODEL_VERSIONS[@]}"
  do
    ROS_DISTRO="$VERSION"
    HF_VERSION="$MODEL_NAME-$MODEL_VERSION"
    TAG="$MODEL_VERSION"
    gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_VERSION="$ROS_DISTRO",_TAG="$TAG",_MODEL_VERSION="$HF_VERSION",_ORGANIZATION="$ORGANIZATION" --timeout=10000 &
    pids+=($!)
    echo Dispatched "$MODEL_VERSION" on ROS "$ROS_DISTRO"
  done
done

for pid in ${pids[*]}; do
  wait "$pid"
done

echo "All builds finished"
