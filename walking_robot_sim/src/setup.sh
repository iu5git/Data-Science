#!/bin/bash
# setup.sh - WalkingRobotSim Initial Setup v2.6

set -e

# ════════════════════════════════════════════════════════════
# ЦВЕТА
# ════════════════════════════════════════════════════════════

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ════════════════════════════════════════════════════════════
# ФУНКЦИИ
# ════════════════════════════════════════════════════════════

info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

success() {
    echo -e "${GREEN}[✓]${NC} $*"
}

error() {
    echo -e "${RED}[✗]${NC} $*" >&2
}

warning() {
    echo -e "${YELLOW}[!]${NC} $*"
}

# ════════════════════════════════════════════════════════════
# ПРОВЕРКИ
# ════════════════════════════════════════════════════════════

check_docker() {
    info "Проверка Docker..."
    if ! command -v docker &> /dev/null; then
        error "Docker не установлен"
        echo "Установите Docker: https://docs.docker.com/get-docker/"
        exit 1
    fi
    success "Docker найден: $(docker --version)"
}

check_docker_compose() {
    info "Проверка Docker Compose..."
    if ! docker compose version &> /dev/null 2>&1; then
        error "Docker Compose не установлен"
        echo "Установите Docker Compose: https://docs.docker.com/compose/install/"
        exit 1
    fi
    success "Docker Compose найден: $(docker compose version --short)"
}

check_project_structure() {
    info "Проверка структуры проекта..."
    
    PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    local required_files=(
        "docker/Dockerfile"
        "docker/compose.yml"
        "docker/manage.sh"
        "docker/cyclonedds.xml"
    )
    
    for file in "${required_files[@]}"; do
        if [ ! -f "$PROJECT_ROOT/$file" ]; then
            error "Файл не найден: $file"
            exit 1
        fi
    done
    
    success "Структура проекта верна"
}

check_permissions() {
    info "Проверка прав доступа..."
    
    PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    if [ ! -x "$PROJECT_ROOT/docker/manage.sh" ]; then
        warning "manage.sh не имеет прав на выполнение. Исправляю..."
        chmod +x "$PROJECT_ROOT/docker/manage.sh"
        success "Права установлены"
    else
        success "Права доступа корректны"
    fi
}

check_x11() {
    info "Проверка X11 (для GUI)..."
    
    if [ -z "$DISPLAY" ]; then
        warning "DISPLAY не установлен. X11 GUI может не работать."
        info "Для использования GUI установите DISPLAY:"
        info "  export DISPLAY=:0"
        info "  xhost +local:"
    else
        success "DISPLAY установлен: $DISPLAY"
    fi
}

show_system_info() {
    info "Информация о системе:"
    echo "  OS: $(uname -s)"
    echo "  Kernel: $(uname -r)"
    echo "  Docker: $(docker --version)"
    echo "  Compose: $(docker compose version --short)"
    echo "  User: $(whoami)"
    echo "  Home: $HOME"
}

# ════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════

main() {
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║  🤖 WalkingRobotSim v2.6 - Setup                           ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    check_docker
    check_docker_compose
    check_project_structure
    check_permissions
    check_x11
    
    echo ""
    show_system_info
    
    echo ""
    success "✅ Инициализация завершена успешно!"
    
    echo ""
    info "Следующие шаги:"
    echo "  1. Перейти в директорию: cd docker"
    echo "  2. Собрать образ: ./manage.sh build"
    echo "  3. Запустить контейнер: ./manage.sh up-bg"
    echo "  4. Запустить Gazebo: ./manage.sh gazebo"
    echo "  5. Управлять роботом: ./manage.sh teleop (в другом терминале)"
    echo ""
    echo "  Справка: ./manage.sh --help"
    echo ""
}

main "$@"
