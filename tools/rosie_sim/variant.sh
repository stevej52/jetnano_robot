#!/usr/bin/env bash
# variant.sh NAME 'python-expr-on-p'... : build nav2_sim.yaml from nav2_rosie.yaml + edits, restart the sim
# e.g. ./variant.sh A "p['controller_server']['ros__parameters']['FollowPath']['max_allowed_time_to_collision_up_to_carrot']=0.5"
cd ~/rosie-sim
source env.sh
NAME=$1; shift
python3 - "$@" <<'EOF'
import sys, yaml
src = open('nav2_rosie.yaml').read().replace('__BT_DIR__', '/home/steve/rosie-sim/bt')
p = yaml.safe_load(src)
for e in sys.argv[1:]:
    exec(e)
yaml.safe_dump(p, open('nav2_sim.yaml', 'w'), sort_keys=False)
EOF
for pid in $(ps -eo pid,args | awk '$2 ~ /^(python3|\/usr\/bin\/python3|\/opt\/ros\/jazzy\/lib\/nav2)/ && /rosie-sim|sim.launch|nav2_/ {print $1}'); do
    [ "$pid" != "$$" ] && kill -INT "$pid" 2>/dev/null
done
sleep 6
for pid in $(ps -eo pid,args | awk '$2 ~ /^(python3|\/usr\/bin\/python3|\/opt\/ros\/jazzy\/lib\/nav2)/ && /rosie-sim|sim.launch|nav2_/ {print $1}'); do
    kill -KILL "$pid" 2>/dev/null
done
setsid ros2 launch ./sim.launch.py > out/launch-$NAME.log 2>&1 < /dev/null &
for i in $(seq 1 30); do
    sleep 2
    grep -q "Managed nodes are active" out/launch-$NAME.log && { echo "variant $NAME: Nav2 active"; exit 0; }
done
echo "variant $NAME: Nav2 NOT active"; tail -5 out/launch-$NAME.log; exit 1
