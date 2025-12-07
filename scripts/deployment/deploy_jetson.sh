#!/usr/bin/env bash
#
# Automated Jetson Orin Deployment Script
# Deploys capstone demo Docker container to Jetson hardware
#
# Usage: ./deploy_jetson.sh [jetson-hostname] [--build]
# Example: ./deploy_jetson.sh jetson-orin.local --build
#
# Author: GIAIC Hackathon Q4 Team
# License: MIT

set -e  # Exit on error

# Configuration
JETSON_HOST="${1:-jetson-orin.local}"
JETSON_USER="${JETSON_USER:-jetson}"
CONTAINER_NAME="capstone-jetson"
IMAGE_NAME="capstone-jetson:latest"
BUILD_FLAG="$2"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."

    # Check if Docker is installed locally
    if ! command -v docker &> /dev/null; then
        log_error "Docker not found. Please install Docker."
        exit 1
    fi

    # Check if SSH key is set up for Jetson
    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 ${JETSON_USER}@${JETSON_HOST} true 2>/dev/null; then
        log_warn "SSH key not set up for ${JETSON_HOST}. You'll need to enter password."
    fi

    log_info "Prerequisites check passed ✓"
}

# Build Docker image (optional)
build_image() {
    if [ "$BUILD_FLAG" == "--build" ]; then
        log_info "Building Docker image for ARM64..."

        # Check if buildx is available
        if docker buildx version &> /dev/null; then
            docker buildx build \
                --platform linux/arm64 \
                -t ${IMAGE_NAME} \
                -f examples/module-05-capstone/docker/Dockerfile.jetson \
                .
        else
            log_warn "Docker buildx not available. Building native image..."
            docker build \
                -t ${IMAGE_NAME} \
                -f examples/module-05-capstone/docker/Dockerfile.jetson \
                .
        fi

        log_info "Docker image built successfully ✓"
    else
        log_info "Skipping build (use --build flag to rebuild)"
    fi
}

# Save and transfer Docker image
transfer_image() {
    log_info "Saving Docker image to tarball..."
    docker save ${IMAGE_NAME} | gzip > /tmp/${CONTAINER_NAME}.tar.gz

    log_info "Transferring image to Jetson (this may take several minutes)..."
    scp /tmp/${CONTAINER_NAME}.tar.gz ${JETSON_USER}@${JETSON_HOST}:/tmp/

    log_info "Loading image on Jetson..."
    ssh ${JETSON_USER}@${JETSON_HOST} "docker load < /tmp/${CONTAINER_NAME}.tar.gz"

    # Cleanup
    rm /tmp/${CONTAINER_NAME}.tar.gz
    ssh ${JETSON_USER}@${JETSON_HOST} "rm /tmp/${CONTAINER_NAME}.tar.gz"

    log_info "Image transfer complete ✓"
}

# Deploy container on Jetson
deploy_container() {
    log_info "Deploying container on Jetson..."

    # Stop existing container if running
    ssh ${JETSON_USER}@${JETSON_HOST} "docker stop ${CONTAINER_NAME} 2>/dev/null || true"
    ssh ${JETSON_USER}@${JETSON_HOST} "docker rm ${CONTAINER_NAME} 2>/dev/null || true"

    # Run new container
    ssh ${JETSON_USER}@${JETSON_HOST} << 'EOF'
docker run -d \
  --name capstone-jetson \
  --runtime=nvidia \
  --restart=unless-stopped \
  --network=host \
  --device=/dev/snd:/dev/snd \
  --device=/dev/video0:/dev/video0 \
  --privileged \
  -e OPENAI_API_KEY="${OPENAI_API_KEY}" \
  -e ROS_DOMAIN_ID=42 \
  -v /home/jetson/models:/models:ro \
  -v /home/jetson/capstone_data:/data \
  -v /home/jetson/capstone_logs:/logs \
  capstone-jetson:latest \
  bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && tail -f /dev/null"
EOF

    log_info "Container deployed successfully ✓"
}

# Verify deployment
verify_deployment() {
    log_info "Verifying deployment..."

    # Check if container is running
    if ssh ${JETSON_USER}@${JETSON_HOST} "docker ps | grep ${CONTAINER_NAME}" &> /dev/null; then
        log_info "Container is running ✓"
    else
        log_error "Container failed to start"
        exit 1
    fi

    # Check ROS 2 nodes
    log_info "Checking ROS 2 environment..."
    ssh ${JETSON_USER}@${JETSON_HOST} "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'" || true

    log_info "Deployment verification complete ✓"
}

# Print usage instructions
print_instructions() {
    log_info "=========================================="
    log_info "Deployment Complete!"
    log_info "=========================================="
    echo ""
    echo "To access the container:"
    echo "  ssh ${JETSON_USER}@${JETSON_HOST}"
    echo "  docker exec -it ${CONTAINER_NAME} bash"
    echo ""
    echo "To start the capstone demo:"
    echo "  ros2 launch capstone_demo gazebo_demo.launch.py"
    echo ""
    echo "To monitor resources:"
    echo "  python3 /ros2_ws/src/capstone_demo/scripts/deployment/monitor_resources.py"
    echo ""
    echo "To view logs:"
    echo "  docker logs -f ${CONTAINER_NAME}"
    echo ""
}

# Main execution
main() {
    log_info "Starting Jetson deployment to ${JETSON_HOST}..."

    check_prerequisites
    build_image
    transfer_image
    deploy_container
    verify_deployment
    print_instructions

    log_info "Deployment script completed successfully! 🚀"
}

# Run main function
main "$@"
