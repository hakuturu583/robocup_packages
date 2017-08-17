export NAO_IP=169.254.199.42
export ROSCORE_IP=127.0.0.1
export TEAM_COLOR=R
export TEAM_NUMBER=51
export PLAYER_NUMBER=1
echo NAO_IP setted: $NAO_IP
echo ROSCORE_IP setted: $ROSCORE_IP
echo TEAM_COLOR setted: $TEAM_COLOR
echo TEAM_NUMBER setted: $TEAM_NUMBER
echo PLAYER_NUMBER setted: $PLAYER_NUMBER

function play_bag() {
  roslaunch robocup_launcher play.launch on_real_robot:=false play_bag:=true
}

function play_on_real_robot() {
  roslaunch robocup_launcher play.launch on_real_robot:=true play_bag:=false
}
