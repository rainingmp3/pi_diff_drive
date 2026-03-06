Docker image of the project with nvim config injected in it, delete nvim part if no needed
build.sh - build container; 
run.sh - run container; 

Main comands:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
                               --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
