export REPOSITORY_NAME=${PWD##*/}
echo "Testing branch '$TRAVIS_BRANCH' of '$REPOSITORY_NAME' on ROS '$ROS_DISTRO'"

# Start Docker container
cp ~/.ssh/id_rsa id_rsa && docker build -t $DOCKER_IMAGE -f Dockerfile.ci $CI_SOURCE_PATH
docker_result=$?
rm id_rsa

# Check if docker ran successfully
if [ $docker_result -ne 0 ]; then
  echo "$DOCKER_IMAGE container finished with errors"
  exit 1 # error
fi
echo "$DOCKER_IMAGE container finished successfully"
exit 0