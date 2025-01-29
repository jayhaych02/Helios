#!/bin/bash

# Set strict error handling
set -euo pipefail
IFS=$'\n\t'

# Constants
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly LOG_FILE="${SCRIPT_DIR}/git_lfs_installation.log"

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

# Install Git LFS
install_git_lfs() {
    log_info "Installing Git LFS..."

    # Add packagecloud repository
    if ! curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash; then
        log_error "Failed to add Git LFS repository"
        exit 1
    fi

    # Install git-lfs package
    if ! sudo apt-get install -y git-lfs; then
        log_error "Failed to install Git LFS"
        exit 1
    fi

    # Initialize Git LFS
    if ! git lfs install; then
        log_error "Failed to initialize Git LFS"
        exit 1
    fi

    log_success "Git LFS installation completed"
}

# Configure Git LFS tracking
configure_git_lfs() {
    log_info "Configuring Git LFS file tracking..."

    # Unity files
    git lfs track "*.fbx"
    git lfs track "*.obj"
    git lfs track "*.unity"
    git lfs track "*.prefab"
    git lfs track "*.asset"

    # Gazebo/ROS files
    git lfs track "*.dae"
    git lfs track "*.stl"
    git lfs track "*.urdf"
    git lfs track "*.world"
    git lfs track "*.sdf"

    # General large files
    git lfs track "*.png"
    git lfs track "*.jpg"
    git lfs track "*.jpeg"
    git lfs track "*.psd"
    git lfs track "*.wav"
    git lfs track "*.mp3"

    log_success "Git LFS tracking configured"
}

# Main installation process
main() {
    # Clear or create new log file
    : > "${LOG_FILE}"
    
    log_info "Starting Git LFS installation..."
    
    # Install and configure Git LFS
    install_git_lfs
    configure_git_lfs

    log_success "Git LFS setup completed successfully!"
    log_info "Check ${LOG_FILE} for detailed installation logs"
}

# Run main function
main
