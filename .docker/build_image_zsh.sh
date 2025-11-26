
echo -e "Building image deco2_mosaic:latest"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target zsh \
--tag deco2_mosaic:latest .