Docker image of the project with nvim config injected in it, delete nvim part if no needed
build.sh - build container; 
run.sh - run container; 

Main comands:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
 colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

Also: mv pi_diff_drive/ src/ 
CompileFlags:
  Add:
    - "-isystem/usr/include/c++/13"
    - "-isystem/usr/include/x86_64-linux-gnu/c++/13"
    - "-isystem/usr/include/c++/13/backward"
    - "-isystem/usr/lib/gcc/x86_64-linux-gnu/13/include"
    - "-isystem/usr/local/include"
    - "-isystem/usr/include/x86_64-linux-gnu"
    - "-isystem/usr/include"
