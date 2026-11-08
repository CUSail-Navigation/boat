# Dockerized ROS 2 Boat Setup

This tutorial will guide you through the steps to clone our ROS 2 project repository via SSH, build a Docker image, and 
run it in a Docker container. The container uses **ROS 2 Humble Hawksbill on Ubuntu 22.04 (Jammy Jellyfish)**. 
Note that the Ubuntu version matches JetPack 6.

---

## Prerequisites

Before we start, ensure the following prerequisites are met:
1. **Docker** is installed on your machine. You can download Docker Desktop [here](https://www.docker.com/products/docker-desktop).
2. **Git** is installed on your machine.
3. You have **SSH keys** set up and added to your GitHub account. If not, follow [GitHub's guide on SSH key setup](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh).

---

## Step 1: Clone the Repository via SSH

1. Open your terminal and navigate to the directory where you want to clone the repository.
   ```bash
   cd ~/path/to/your/workspace
   ```
2. Clone the repository using SSH. Use the `boat` repository URL:
   ```bash
   git clone git@github.com:CUSail-Navigation/boat.git
   ```
   This will create a directory with the project files. Navigate to this directory:
   ```bash
   cd boat
   ```

Before building the Docker image, let's quickly go over the project structure. This project has two main directories: 
`setup/` and `src/`.

- The `setup/` folder contains the `Dockerfile`, `requirements.txt`, and `setup_ros.sh` script used to configure your 
  ROS 2 environment in Docker.
- The `src/` folder holds the boat’s ROS 2 packages and code.

### **Explanation of files:**
- **`/setup/Dockerfile`**: defines the Docker image used to create a containerized ROS 2 environment. This includes 
  setting up the base image, installing dependencies, and configuring the workspace.
- **`/setup/requirements.txt`**: contains a list of Python dependencies (if any) that will be installed inside the 
  Docker container using `pip`. This file ensures that all necessary Python libraries are installed before running the 
  ROS 2 packages.
- **`/setup/setup_ros.sh`**: a shell script that automates the setup of your ROS 2 workspace. It builds the ROS 2 
  packages, sources the workspace, and keeps the container running interactively to test your code quicker.
- **`/src/`**: this directory holds your ROS 2 packages, which contain the nodes, launch files, and any other necessary 
  code for your ROS 2 application. This is where the core functionality of our project resides.

With this structure, `setup/` contains the environment configuration and `src/` holds the boat’s ROS 2 codebase. The 
`src/` directory is mounted into the Docker container for development.

---

## Step 1.5: Ensure Unix Style Line Endings (`setup_ros.sh`)

To avoid issues with line endings that may prevent `setup_ros.sh` from running properly in the Docker container, keep 
the file saved with Unix-style (LF) line endings. If it was converted to Windows-style (CRLF), this command works on 
macOS and Linux:
```bash
perl -pi -e 's/\r$//' setup/setup_ros.sh
```

---

## Step 2: Build the Docker Image

Start Docker Desktop, and then run this command from the repository root:
   ```bash
   docker build -t boat-local setup
   ```
   - `-t boat-local`: tags the image as `boat-local`.
   - `setup`: tells Docker to use the `setup/` directory as the build context.

Note that Docker will use the `Dockerfile` to create the image. This step may take a few minutes as Docker installs 
dependencies and sets up the environment. If you get an error like `ERROR: Cannot connect to the Docker daemon`, launch 
the Docker GUI application you installed and run this command again.

---

## Step 3: Run the Docker Container

Once the image is built, you can run the Docker container. Here's how to do it:

### **Run the container with volume mounting:**
Run the following command from the project root to mount `src/` into the container workspace:
```bash
docker run -it --rm --name boat-dev -v "$(pwd)/src:/home/ros2_user/ros2_ws/src" boat-local
```
- `-it`: runs the container interactively, allowing you to enter commands.
- `--rm`: automatically removes the container when it exits.
- `--name boat-dev`: names the container `boat-dev`.
- `-v "$(pwd)/src:/home/ros2_user/ros2_ws/src"`: mounts the `src/` directory from your host machine into the container 
  at `/home/ros2_user/ros2_ws/src`.
- `boat-local`: the name of the Docker image you built in Step 2.

The run command builds and sources the mounted ROS 2 workspace, and then opens a Bash shell inside the container. Build
failures stop startup. You can run `ros2` commands in  this shell; ROS 2 does not need to be installed on your host.

### Verify the installation:
In the container shell opened above, run the package tests and inspect the results:
```bash
colcon test --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

The results should report zero errors and zero failures. The included `smoke`
package checks that a ROS 2 node can publish and receive a message.

---

## Step 4: Understanding our Software Development Lifecycle (SDLC) Process

In this setup, the development process follows a typical Software Development Lifecycle (SDLC) workflow where you write 
and test your code iteratively. Here’s how it works with your ROS 2 project:
1. **Developing code in the `boat/src` directory:**
   - You will write and edit your code directly in the `boat/src` directory, which contains your ROS 2 packages. This 
     is where all of your nodes, launch files, and configurations reside.
   - You are free to use any IDE or text editor you prefer to edit and manage your code. Since this directory is part of 
     the local file system, your changes will be saved locally.
2. **Testing your code in the Docker container:**
   - Once you have made changes and want to test them, you don’t need to worry about copying files into the container 
     manually. The development command mounts your local `src/` directory into the Docker container.
   - To start testing, simply run the Docker image with the command we defined earlier:
     ```bash
     docker run -it --rm --name boat-dev -v "$(pwd)/src:/home/ros2_user/ros2_ws/src" boat-local
     ```
   - This will start the Docker container, mounting the `src/` directory from your local machine, and build the 
     workspace inside the container. From here, you can run any ROS 2 nodes you wish to test.
3. **Live changes and ROS node rebuild:**
   - Any modifications you make in the local `boat/src` directory will automatically be reflected inside the Docker 
     container because of the mounted directory. This allows you to work and test your code seamlessly.
   - Although live changes are reflected in the container, changes to the code often require you to rebuild your ROS 2 
     workspace inside the container. Run this inside the container shell:
     ```bash
     colcon build
     ```
     After the build, you should source the setup file again:
     ```bash
     source install/setup.bash
     ```
     This ensures that the changes to your nodes or packages are properly built and ready for testing.
4. **Opening a second terminal running the same container:**
   - Sometimes, you may want to enter the same container from a second terminal to run concurrent processes. 
     Run this on your host in a second terminal while the original container is running:
     ```bash
     docker exec -it boat-dev bash
     ```
   Bash loads ROS 2 and the built workspace automatically.