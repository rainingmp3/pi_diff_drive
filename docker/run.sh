docker run -it --rm \
  -w /root/donatello/root_ws \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/ingenuity/Code/ros2/pi_diff_drive/:/root/donatello/root_ws/src"\
  --volume="/home/ingenuity/.config/nvim/:/root/.config/nvim:rw" \
  --volume="/home/ingenuity/.local/share/nvim/:/root/.local/share/nvim:rw" \
  --volume="/home/ingenuity/.cache/nvim/:/root/.cache/nvim:rw" \
  --network=host \
donatello  
