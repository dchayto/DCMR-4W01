#!/bin/bash

# executable command to do a lightweight build of selected ros2 packages
# (for if pi is getting stuck on a build)
if [ "$#" -eq 0 ]; then
    colcon build \
        --parallel-workers 1 \
		--event-handlers status- \
		-- executor sequential \
        --symlink-install
else
	colcon build \
		--parallel-workers 1 \
		--event-handlers status- \
		-- executor sequential \
        --symlink-install
		--packages-select "$@"
fi
