#!/usr/bin/env sh

session_name="ros2_lifecycle_demo"

tmux has-session -t $session_name

# Validate if session is already there
if [ $? != 0 ]; then
    # Create a new session
    tmux new-session -s $session_name -n $session_name -d

    # Lifecycle Talker
    tmux send-keys -t $session_name 'ross2; ros2 launch ros2-lifecycle start_lifecycle_talker.py' C-m

    # Normal Listener
    tmux split-window -v -t $session_name
    tmux send-keys -t $session_name 'ross2; ros2 launch ros2-lifecycle start_normal_listener.py' C-m

    # Lifecycle Client
    tmux split-window -h -t $session_name
    tmux send-keys -t $session_name 'ross2; ros2 launch ros2-lifecycle start_lifecycle_client.py' C-m

    # Select tiled layout
    tmux select-layout tiled
fi

tmux attach -t $session_name