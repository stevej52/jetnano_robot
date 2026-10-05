#!/usr/bin/env bash
# variant.sh NAME 'python-expr-on-p'... : build nav2_sim.yaml from nav2_rosie.yaml + edits, restart THIS
# instance's sim (the directory this script is in; several instances can run side by side, each with
# its own ROS domain in env.sh).
# e.g. ./variant.sh A "p['controller_server']['ros__parameters']['FollowPath']['use_collision_detection']=False"
cd "$(dirname "$0")"
D=$(pwd)
source env.sh
NAME=$1; shift
python3 - "$D" "$@" <<'EOF'
import sys, yaml
d = sys.argv[1]
src = open('nav2_rosie.yaml').read().replace('__BT_DIR__', d + '/bt')
p = yaml.safe_load(src)
for e in sys.argv[2:]:
    exec(e.replace('{D}', d))
yaml.safe_dump(p, open('nav2_sim.yaml', 'w'), sort_keys=False)
EOF
# stop this instance only: its launch runs in its own process group (setsid), recorded below
if [ -f out/launch.pgid ]; then
    PG=$(cat out/launch.pgid)
    kill -INT -- -"$PG" 2>/dev/null
    for i in $(seq 1 10); do pgrep -g "$PG" >/dev/null || break; sleep 1; done
    kill -KILL -- -"$PG" 2>/dev/null
    rm -f out/launch.pgid
fi
setsid bash -c 'echo $$ > out/launch.pgid; exec ros2 launch ./sim.launch.py' > out/launch-$NAME.log 2>&1 < /dev/null &
for i in $(seq 1 30); do
    sleep 2
    grep -q "Managed nodes are active" out/launch-$NAME.log && { echo "variant $NAME: Nav2 active ($D, domain $ROS_DOMAIN_ID)"; exit 0; }
done
echo "variant $NAME: Nav2 NOT active"; tail -5 out/launch-$NAME.log; exit 1
