#!/bin/bash
# Pick the robot's start position and the order to visit the other four positions, then open
# Gazebo (gz_ca2_start.sh) and, 3 seconds later, the path planner (ca2.sh) in their own terminals.
cd "$(dirname "$(readlink -f "$0")")" || exit 1
X=(0.0 3.2 3.4 2.4 -0.4)
Y=(0.0 0.2 -3.6 -3.6 -3.8)

echo "Positions:"
for i in 0 1 2 3 4; do echo "  $i = (${X[$i]}, ${Y[$i]})"; done

while true; do
    read -r -p "Start position (0-4): " START || exit 1
    [[ "$START" =~ ^[0-4]$ ]] && break
    echo "  Enter one of 0 1 2 3 4"
done

ALLOWED=$(echo 01234 | tr -d "$START") # e.g. 1234 when the start is 0
while true; do
    read -r -p "Goal order, e.g. $ALLOWED (use each of $ALLOWED once): " ORDER || exit 1
    ORDER=$(echo "$ORDER" | tr -d ' ')
    [ "$(echo "$ORDER" | grep -o . | sort | tr -d '\n')" == "$ALLOWED" ] && break
    echo "  Enter each of $ALLOWED exactly once, e.g. $ALLOWED"
done

GOAL_LIST=""
for g in $(echo "$ORDER" | grep -o .); do GOAL_LIST+="${GOAL_LIST:+, }(${X[$g]}, ${Y[$g]})"; done
sed -i -E "s/^    \"goal_list\": \[.*\],$/    \"goal_list\": [$GOAL_LIST],/" src/rb2301_ca2/rb2301_ca2/path_planning.py
echo "Start: (${X[$START]}, ${Y[$START]})"
grep -E '^    "goal_list": \[' src/rb2301_ca2/rb2301_ca2/path_planning.py

gnome-terminal --title="CA2 Gazebo" --working-directory="$PWD" -- bash -c "./gz_ca2_start.sh ${X[$START]} ${Y[$START]}; exec bash"
sleep 3
gnome-terminal --title="CA2 path planning" --working-directory="$PWD" -- bash -c "./ca2.sh; exec bash"
