# ROS 2 Humble Docker Tutorial

A simple Docker setup for running ROS 2 Humble tutorials with a basic publisher/subscriber example using C++ and Python scripts.

## Repository Structure

```
.
├── docker
│   ├── docker-compose.yml
│   ├── Dockerfile
│   └── entrypoint.sh
├── README.md
└── ros2_ws
    └── src
        ├── cpp_pubsub
        │   ├── CMakeLists.txt
        │   ├── LICENSE
        │   ├── package.xml
        │   └── src
        │       ├── publisher_member_function.cpp
        │       └── subscriber_member_function.cpp
        └── py_pubsub
            ├── LICENSE
            ├── package.xml
            ├── py_pubsub
            │   ├── __init__.py
            │   ├── publisher_member_function.py
            │   └── subscriber_member_function.py
            ├── resource
            │   └── py_pubsub
            ├── setup.cfg
            ├── setup.py
            └── test
                ├── test_copyright.py
                ├── test_flake8.py
                └── test_pep257.py
```

## What's Included

- **Dockerfile**: Simplified ROS Noetic container setup
- **docker-compose.yml**: Container orchestration configuration
- **cpp_pubsub package**: Contains the classic ROS 2 tutorial scripts
  - `talker`: Publisher node that sends "hello world" messages
  - `listener`: Subscriber node that receives and logs messages
- **py_pubsub package**: Contains the classic ROS 2 tutorial scripts
  - `talker`: Publisher node that sends "hello world" messages
  - `listener`: Subscriber node that receives and logs messages

## Prerequisites

- Docker installed (and opened) on your system

## Quick Start

### 0. Clone and Enter Repo Directory

Open your terminal and navigate to a directory where you want to save locally save this repository.

```bash
# Clone this repository
git clone https://github.com/usc-robosub/ros-tutorials.git

# Enter the repository
cd ros-tutorials
```

### 1. Build and Start the Container

```bash
# Enter the docker directory
cd docker

# Build and start the container in detached mode
docker compose up -d
```

This will:

- Build the Docker image with ROS 2 Humble and your colcon workspace
- Start the container named `ros2-humble-tutorial`
- Run in the background (detached mode)

### 2. Set Up Three Terminal Sessions

You'll need two separate terminal windows/tabs for this demo.

#### Terminal 1: Start the Listener Node

Open a new terminal window/tab:

```bash
# Connect to the container in a new session
docker exec -it ros2-humble-tutorial bash
```

```bash
# Run the listener node from the cpp_pubsub package
ros2 run cpp_pubsub listener
```

The listener is now waiting for messages. You should see a blank terminal waiting for a response from the Talker Node.

#### Terminal 2: Start the Talker Node

Open a third terminal window/tab:

```bash
# Connect to the container in a new session
docker exec -it ros2-humble-tutorial bash
```

```bash
# Run the talker node from the py_pubsub package
ros2 run py_pubsub talker
```

You should now see:

- **Terminal 2 (talker)**: Publishing "hello world" messages
- **Terminal 1 (listener)**: Receiving and displaying those messages

## Expected Output

**Terminal 2 (Talker):**

```
[INFO] [1772491778.921476429] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1772491779.408108096] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1772491779.907574222] [minimal_publisher]: Publishing: "Hello World: 2"
```

**Terminal 1 (Listener):**

```
[INFO] [1772491778.906705804] [minimal_subscriber]: I heard: 'Hello World: 0'
[INFO] [1772491779.407363763] [minimal_subscriber]: I heard: 'Hello World: 1'
[INFO] [1772491779.907296013] [minimal_subscriber]: I heard: 'Hello World: 2'
```

## Safely Exiting

### Step 1: Stop the ROS 2 Nodes

In each terminal (in reverse order):

1. **Terminal 2 (talker)**: Press `Ctrl+C` to stop the talker node
2. **Terminal 1 (listener)**: Press `Ctrl+C` to stop the listener node

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

If you want to modify the Python or C++ scripts without rebuilding the container, use the `dev` profile when starting up the container

```bash
# Build and start the dev container in detached mode
docker compose --profile dev up -d
```

Now two images and containers have been made, make sure to enter the `ros2-humble-tutorial-dev` container

```bash
# Connect to the container
docker exec -it ros2-humble-tutorial bash
```

After making edits to any python scripts within the directories with `catkin_ws/src` on your local computer, make sure to rebuild resource the ros workspace

```bash
# Change back to the root of the workspace
cd /ros2_ws

# Rebuild the ros 2 workspace
colcon build --symlink-install
```

```bash
# Resource the new build into the current workspace
source install/setup.bash
```

When finished using and exiting the dev container, run the following command from your local machine

```bash
# Stop and remove the containers, networks, and orphaned containers
docker compose --profile dev down --rmi all --remove-orphans
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

## Troubleshooting

### Container Won't Start

```bash
# Check container status
docker-compose ps

# View detailed logs
docker-compose logs ros2-humble-tutorial
docker-compose logs ros2-humble-tutorial-dev
```

### ROS 2 Nodes Can't Communicate

- Ensure all three terminals are connected to the same container
- Check that the container is using `network_mode: host`

## What's Next?

- Modify the Python or C++ scripts to experiment with different message types
- Add more ROS 2 nodes to the package
- Explore ROS topics with `ros2 topic list` and `ros2 topic echo /topic`
