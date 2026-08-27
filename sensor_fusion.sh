#!/usr/bin/env bash

set -u

# Directory containing this script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COMPOSE_FILE="$SCRIPT_DIR/compose.yaml"
SERVICE_NAME="open-pi-sensor-fusion"

LOG_DIR="$SCRIPT_DIR/../../script_logs"
LOG_FILE="$LOG_DIR/sensor_fusion.txt"

usage() {
    echo "Usage: $0 COMMAND"
    echo
    echo "Commands:"
    echo "  build                Build the Docker image only"
    echo "  create,  -c          Build, create and start the container"
    echo "  start,   -s          Start the existing container"
    echo "  stop,    -t          Stop the container"
    echo "  restart, -r          Restart the container"
    echo "  remove,  -x          Stop and remove the container"
    echo "  status               Show the container status"
    echo "  check                Check module prerequisites"
    echo "  logs                 Follow the container logs"
    echo "  help,    -h          Show this help message"
}

die() {
    local message="$1"

    mkdir -p "$LOG_DIR"

    echo "[ERROR] $message" >&2
    echo "[ERROR] $message" >> "$LOG_FILE"

    exit 1
}

compose() {
    docker compose \
        --file "$COMPOSE_FILE" \
        "$@"
}

check_dependencies() {
    if ! command -v docker >/dev/null 2>&1; then
        die "Docker is not installed"
    fi

    if ! docker compose version >/dev/null 2>&1; then
        die "Docker Compose is not available"
    fi

    if [[ ! -f "$COMPOSE_FILE" ]]; then
        die "Compose file not found: $COMPOSE_FILE"
    fi

    if ! docker info >/dev/null 2>&1; then
        die "Docker daemon is not reachable"
    fi
}



build_image() {
    check_dependencies

    echo "[INFO] Building sensor_fusion image"

    compose build "$SERVICE_NAME"
}

create_container() {
    check_dependencies
    

    echo "[INFO] Building and starting sensor_fusion container"

    compose up --detach --build "$SERVICE_NAME"
}

start_container() {
    check_dependencies
    

    echo "[INFO] Starting sensor_fusion container"

    compose start "$SERVICE_NAME"
}

stop_container() {
    check_dependencies

    echo "[INFO] Stopping sensor_fusion container"

    compose stop "$SERVICE_NAME"
}

restart_container() {
    check_dependencies
    

    echo "[INFO] Restarting sensor_fusion container"

    compose restart "$SERVICE_NAME"
}

remove_container() {
    check_dependencies

    echo "[INFO] Stopping and removing sensor_fusion container"

    compose rm --stop --force "$SERVICE_NAME"
}

status_container() {
    check_dependencies

    compose ps "$SERVICE_NAME"
}

check_module() {
    check_dependencies
    

    echo "[OK] Docker is available"
    echo "[OK] Docker daemon is running"
    echo "[OK] Compose file found"
}

show_logs() {
    check_dependencies

    compose logs --follow "$SERVICE_NAME"
}

main() {
    if [[ $# -ne 1 ]]; then
        usage
        return 2
    fi

    case "$1" in
        build|--build)
            build_image
            ;;

        create|-c|--create)
            create_container
            ;;

        start|-s|--start)
            start_container
            ;;

        stop|-t|--stop)
            stop_container
            ;;

        restart|-r|--restart)
            restart_container
            ;;

        remove|-x|--remove)
            remove_container
            ;;

        status|--status)
            status_container
            ;;

        check|--check)
            check_module
            ;;

        logs|--logs)
            show_logs
            ;;

        help|-h|--help)
            usage
            ;;

        *)
            echo "Unknown command: $1" >&2
            usage
            return 2
            ;;
    esac
}

main "$@"