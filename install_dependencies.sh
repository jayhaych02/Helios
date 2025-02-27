#!/bin/bash

# Set strict error handling
set -euo pipefail
IFS=$'\n\t'

# Constants
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly LOG_FILE="${SCRIPT_DIR}/installation.log"
readonly REQUIRED_UBUNTU_VERSION="22.04"
readonly ROS_DISTRO="humble"

# Color codes for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

# Logging functions
log() {
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${timestamp} - $1" | tee -a "${LOG_FILE}"
}

log_error() {
    log "${RED}ERROR: $1${NC}"
}

log_success() {
    log "${GREEN}SUCCESS: $1${NC}"
}

log_info() {
    log "${BLUE}INFO: $1${NC}"
}

log_warning() {
    log "${YELLOW}WARNING: $1${NC}"
}

# Error handling
handle_error() {
    local line_no=$1
    local error_code=$2
    log_error "Error occurred in script at line: ${line_no}"
    log_error "Error code: ${error_code}"
    log_error "Please check ${LOG_FILE} for more details"
    exit "${error_code}"
}

# Set up error handling trap
trap 'handle_error ${LINENO} $?' ERR

# Check if script is run with sudo
check_sudo() {
    if [[ $EUID -eq 0 ]]; then
        log_error "This script should not be run with sudo or as root"
        exit 1
    fi
}

# Check directory structure
setup_directories() {
    log_info "Setting up directory structure..."
    
    # Get parent directory of script location
    local parent_dir
    parent_dir="$(dirname "${SCRIPT_DIR}")"
    
    # Verify we're in the correct directory structure
    if [[ "${parent_dir}" != *"/robotics" ]]; then
        log_error "This repository must be cloned inside a 'robotics' directory"
        log_error "Please follow these steps:"
        log_error "1. mkdir robotics"
        log_error "2. cd robotics"
        log_error "3. git clone <repository-url>"
        log_error "4. cd <repository-name>"
        log_error "5. ./install.sh"
        exit 1
    fi
    
    log_success "Directory structure verified"
}

# Check system requirements
check_system_requirements() {
    log_info "Checking system requirements..."
    
    # Check Ubuntu version
    if ! command -v lsb_release >/dev/null 2>&1; then
        log_error "lsb_release command not found. Is this a Ubuntu system?"
        exit 1
    fi

    local ubuntu_version
    ubuntu_version=$(lsb_release -rs)
    if [[ "${ubuntu_version}" != "${REQUIRED_UBUNTU_VERSION}" ]]; then
        log_error "This script requires Ubuntu ${REQUIRED_UBUNTU_VERSION}. Found version: ${ubuntu_version}"
        exit 1
    fi

    # Check available disk space (minimum 10GB)
    local available_space
    available_space=$(df -BG "${HOME}" | awk 'NR==2 {print $4}' | sed 's/G//')
    if ((available_space < 10)); then
        log_error "Insufficient disk space. At least 10GB required, found ${available_space}GB"
        exit 1
    fi

    # Check internet connectivity
    if ! ping -c 1 packages.ros.org >/dev/null 2>&1; then
        log_error "No internet connection or ROS packages server is unreachable"
        exit 1
    fi

    log_success "System requirements check passed"
}

# Install ROS2 Humble
install_ros2() {
    log_info "Installing ROS2 ${ROS_DISTRO^}..."
    
    # Check if ROS2 is already installed
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        log_warning "ROS2 ${ROS_DISTRO^} appears to be already installed"
        return 0
    fi

    # Add ROS2 repository and keys
    if ! sudo apt-get update; then
        log_error "Failed to update package lists"
        exit 1
    fi

    if ! sudo apt-get install -y software-properties-common curl gnupg lsb-release; then
        log_error "Failed to install prerequisite packages"
        exit 1
    fi

    if ! sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; then
        log_error "Failed to download ROS GPG key"
        exit 1
    fi

    if ! echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null; then
        log_error "Failed to add ROS2 repository"
        exit 1
    fi

    # Install ROS2 packages
    if ! sudo apt-get update; then
        log_error "Failed to update package lists after adding ROS2 repository"
        exit 1
    fi

    local ros_packages=(
        "ros-${ROS_DISTRO}-desktop"
        "ros-${ROS_DISTRO}-xacro"
        "ros-${ROS_DISTRO}-urdf"
        "ros-${ROS_DISTRO}-robot-state-publisher"
        "ros-${ROS_DISTRO}-joint-state-publisher-gui"
        "ros-${ROS_DISTRO}-controller-manager"
        "ros-${ROS_DISTRO}-ros2-control"
        "ros-${ROS_DISTRO}-ros2-controllers"
        "ros-${ROS_DISTRO}-gazebo-ros2-control"
    )

    for package in "${ros_packages[@]}"; do
        log_info "Installing ${package}..."
        if ! sudo apt-get install -y "${package}"; then
            log_error "Failed to install ${package}"
            exit 1
        fi
    done

    log_success "ROS2 ${ROS_DISTRO^} installation completed"
}


# Install Gazebo Ionic. Supported Sep, 2024 to Sep, 2026
install_gazebo_ionic() {
    log_info "Installing Gazebo Ionic..."

    # Check if Gazebo Ionic is already installed
    if dpkg -l | grep -q "gz-ionic"; then
        log_warning "Gazebo Ionic appears to be already installed"
        return 0
    fi

    # Install prerequisites
    if ! sudo apt-get install -y lsb-release gnupg; then
        log_error "Failed to install Gazebo Ionic prerequisites"
        exit 1
    fi

    # Add Gazebo repository key
    if ! sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; then
        log_error "Failed to download Gazebo GPG key"
        exit 1
    fi

    # Add Gazebo repository
    if ! echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null; then
        log_error "Failed to add Gazebo repository"
        exit 1
    fi

    # Update package lists
    if ! sudo apt-get update; then
        log_error "Failed to update package lists after adding Gazebo repository"
        exit 1
    fi

    # Install Gazebo Ionic
    if ! sudo apt-get install -y gz-ionic; then
        log_error "Failed to install Gazebo Ionic"
        exit 1
    fi

    log_success "Gazebo Ionic installation completed"
}


# Install development tools and dependencies
install_dependencies() {
    log_info "Installing development tools and dependencies..."

    # Install Python3 pip first
    log_info "Installing Python3 pip..."
    if ! sudo apt-get install -y python3-pip; then
        log_error "Failed to install python3-pip"
        exit 1
    fi

    # Install rosdep via pip if apt installation fails
    log_info "Installing python3-rosdep..."
    if ! sudo apt-get install -y python3-rosdep; then
        log_warning "Failed to install python3-rosdep via apt. Attempting installation via pip..."
        if ! pip3 install rosdep; then
            log_error "Failed to install rosdep via pip"
            exit 1
        else
            log_success "Successfully installed rosdep via pip"
        fi
    fi

    local dev_packages=(
        "python3-colcon-common-extensions"
        "python3-vcstool"
        "build-essential"
        "libgtest-dev"
        "libgmock-dev"
        "python3-pytest"
        "git"
        "net-tools"
        "sqlite3"
        "sqlitebrowser"
        "ros-humble-robot-state-publisher"
        "ros-humble-xacro"
        "ros-humble-tf2-tools"
        "ros-humble-tf2-ros"
        
    )

    for package in "${dev_packages[@]}"; do
        log_info "Installing ${package}..."
        if ! sudo apt-get install -y "${package}"; then
            log_error "Failed to install ${package}"
            exit 1
        fi
    done

    # Initialize rosdep
    if ! sudo rosdep init >/dev/null 2>&1; then
        log_warning "rosdep already initialized"
    fi

    if ! rosdep update; then
        log_error "Failed to update rosdep"
        exit 1
    fi

    log_success "Dependencies installation completed"
}

# Setup ROS workspace
setup_workspace() {
    log_info "Setting up ROS workspace..."

    # Create workspace directory structure
    if ! mkdir -p "${SCRIPT_DIR}/src"; then
        log_error "Failed to create workspace src directory"
        exit 1
    fi
    log_success "Workspace setup completed"
}

# Configure environment
configure_environment() {
    log_info "Configuring environment..."

    local bashrc="${HOME}/.bashrc"
    local ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"

    # Add ROS2 setup to bashrc if not already present
    if ! grep -q "source ${ros_setup}" "${bashrc}"; then
        if ! echo "source ${ros_setup}" >> "${bashrc}"; then
            log_error "Failed to add ROS2 setup to .bashrc"
            exit 1
        fi
    fi

    # Add workspace setup
    if ! grep -q "# ROS2 ${ROS_DISTRO^} workspace" "${bashrc}"; then
        {
            echo -e "\n# ROS2 ${ROS_DISTRO^} workspace"
            echo "export ROS_WS=${SCRIPT_DIR}"
            echo 'if [ -f "${ROS_WS}/install/setup.bash" ]; then'
            echo '    source "${ROS_WS}/install/setup.bash"'
            echo 'fi'
        } >> "${bashrc}" || {
            log_error "Failed to add workspace setup to .bashrc"
            exit 1
        }
    fi

    log_success "Environment configuration completed"
}

# Configure WSL2 graphics settings
configure_wsl_graphics() {
    log_info "Configuring WSL2 graphics settings..."

    # Install VcXsrv using winget if in WSL
    if grep -qi microsoft /proc/version; then
        log_info "WSL2 detected. Installing VcXsrv..."
        if ! powershell.exe winget install marha.VcXsrv; then
            log_warning "Failed to install VcXsrv. Please install it manually from: https://sourceforge.net/projects/vcxsrv/"
        fi
    fi

    local bashrc="${HOME}/.bashrc"

    # Add WSL2 graphics configuration if not already present
    if ! grep -q "# WSL2 graphics configuration" "${bashrc}"; then
        {
            echo -e "\n# WSL2 graphics configuration"
            echo "export DISPLAY=:0"
            echo "export LIBGL_ALWAYS_SOFTWARE=1"
            echo "export MESA_GL_VERSION_OVERRIDE=3.3"
            echo "export GZ_GPU_ASYNC=1"
        } >> "${bashrc}" || {
            log_error "Failed to add WSL2 graphics configuration to .bashrc"
            exit 1
        }
    fi

    # Fix runtime directory permissions
    if [[ -d "/run/user/$(id -u)" ]]; then
        if ! sudo chmod 0700 "/run/user/$(id -u)"; then
            log_warning "Failed to set runtime directory permissions. Some graphical applications might not work correctly."
        fi
    fi

    log_success "WSL2 graphics configuration completed"
}


# Main installation process
main() {
    # Clear or create new log file
    : > "${LOG_FILE}"
    
    log_info "Starting ROS2 ${ROS_DISTRO^} installation..."
    log_info "Installation directory: ${SCRIPT_DIR}"
    log_info "Log file: ${LOG_FILE}"

    # Confirm installation
    echo -e "${YELLOW}This script will install ROS2 ${ROS_DISTRO^} and set up your workspace.${NC}"
    echo -e "${YELLOW}Please make sure you have:"
    echo "  - At least 10GB of free disk space"
    echo "  - A STABLE internet connection"
    echo "  - Ubuntu ${REQUIRED_UBUNTU_VERSION} installed${NC}"
    echo -e "\nPress [ENTER] to continue or [CTRL+C] to cancel"
    read -r

    # Run installation steps
    check_sudo
    setup_directories
    check_system_requirements
    install_ros2
    install_gazebo_ionic
    install_dependencies
    setup_workspace
    configure_environment
    configure_wsl_graphics    

    log_success "Installation completed successfully!"
    log_info "Please run: source ~/.bashrc while INSIDE YOUR HOME DIRECTORY or you may encounter errors when testing ROS nodes"
    log_info "For WSL2 users: Install VcXsrv on Windows and launch it before running Gazebo"
    log_info "Check ${LOG_FILE} for detailed installation logs"

}

# Run main function
main