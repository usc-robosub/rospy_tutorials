# ROS Noetic Docker Tutorial

A simple Docker setup for running ROS Noetic tutorials with a basic publisher/subscriber example using Python scripts.

## Repository Structure

```
.
├── Dockerfile
├── docker-compose.yml
├── README.md
└── catkin_ws/
    └── src/
        └── rospy_tutorials/
            ├── CMakeLists.txt
            ├── package.xml
            ├── scripts/
            │   ├── listener.py
            │   └── talker.py
            └── src/
```

## What's Included

- **Dockerfile**: Simplified ROS Noetic container setup
- **docker-compose.yml**: Container orchestration configuration
- **rospy_tutorials package**: Contains the classic ROS tutorial scripts
  - `talker.py`: Publisher node that sends "hello world" messages
  - `listener.py`: Subscriber node that receives and logs messages

## Prerequisites

- Docker installed on your system
- Docker Compose installed on your system

## Quick Start

### 1. Build and Start the Container

```bash
# Build and start the container in detached mode
docker compose up -d --build
```

This will:

- Build the Docker image with ROS Noetic and your catkin workspace
- Start the container named `ros-tutorials`
- Run in the background (detached mode)

### 2. Set Up Three Terminal Sessions

You'll need three separate terminal windows/tabs for this demo.

#### Terminal 1: Start ROS Master (roscore)

```bash
# Connect to the container
docker exec -it ros-tutorials /bin/bash

# Start the ROS master node
roscore
```

Keep this terminal running. You should see:

```
... logging to /root/.ros/log/...
... started core service [/rosout]
```

#### Terminal 2: Start the Listener Node

Open a new terminal window/tab:

```bash
# Connect to the container in a new session
docker exec -it ros-tutorials /bin/bash

# Run the listener script
rosrun rospy_tutorials listener.py
```

The listener is now waiting for messages. You should see:

```
[INFO] [timestamp]: /listener_xxxxx_xxxxx started
```

#### Terminal 3: Start the Talker Node

Open a third terminal window/tab:

```bash
# Connect to the container in a new session
docker exec -it ros-tutorials /bin/bash

# Run the talker script
rosrun rospy_tutorials talker.py
```

You should now see:

- **Terminal 3 (talker)**: Publishing "hello world" messages
- **Terminal 2 (listener)**: Receiving and displaying those messages

## Expected Output

**Terminal 3 (Talker):**

```
[INFO] [1634567890.123]: hello world 1634567890.12
[INFO] [1634567890.223]: hello world 1634567890.22
[INFO] [1634567890.323]: hello world 1634567890.32
```

**Terminal 2 (Listener):**

```
[INFO] [1634567890.124]: /listener_1634567890_123 I heard hello world 1634567890.12
[INFO] [1634567890.224]: /listener_1634567890_123 I heard hello world 1634567890.22
[INFO] [1634567890.324]: /listener_1634567890_123 I heard hello world 1634567890.32
```

## Safely Exiting

### Step 1: Stop the ROS Nodes

In each terminal (in reverse order):

1. **Terminal 3 (talker)**: Press `Ctrl+C` to stop the talker node
2. **Terminal 2 (listener)**: Press `Ctrl+C` to stop the listener node
3. **Terminal 1 (roscore)**: Press `Ctrl+C` to stop roscore

### Step 2: Exit Container Sessions

In each terminal:

```bash
# Exit the container bash session
exit
```

### Step 3: Stop and Clean Up the Container

```bash
# Stop and remove the container, networks, and orphaned containers
docker compose down --remove-orphans --rmi all
```

## Development Tips

### Mounting Workspace for Development

If you want to modify the Python scripts without rebuilding the container, uncomment the volumes section in `docker-compose.yml`:

```yaml
volumes:
  - ./catkin_ws:/catkin_ws
```

Then restart the container:

```bash
docker-compose down
docker-compose up -d
```

### Checking Container Status

```bash
# View running containers
docker ps

# View container logs
docker-compose logs

# View real-time logs
docker-compose logs -f
```

### Alternative: Single Command Execution

Instead of interactive terminals, you can run commands directly:

```bash
# Start roscore in background
docker exec -d ros-tutorials bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roscore"

# Run listener in background
docker exec -d ros-tutorials bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun rospy_tutorials listener.py"

# Run talker interactively
docker exec -it ros-tutorials bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun rospy_tutorials talker.py"
```

## Troubleshooting

### Container Won't Start

```bash
# Check container status
docker-compose ps

# View detailed logs
docker-compose logs ros-tutorials
```

### ROS Nodes Can't Communicate

- Ensure all three terminals are connected to the same container
- Verify roscore is running in Terminal 1 before starting other nodes
- Check that the container is using `network_mode: host`

### Permission Issues

```bash
# If scripts aren't executable, fix permissions
docker exec -it ros-tutorials chmod +x /catkin_ws/src/rospy_tutorials/scripts/*.py
```

## What's Next?

- Modify the Python scripts to experiment with different message types
- Add more ROS nodes to the package
- Try connecting to external ROS masters by modifying `ROS_MASTER_URI`
- Explore ROS topics with `rostopic list` and `rostopic echo /chatter`
