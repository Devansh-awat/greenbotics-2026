#!/bin/bash

# Robot Service Manager for Raspberry Pi 5
# Manages 'open' and 'obstacle' services

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Service names
SERVICE_OPEN="open"
SERVICE_OBSTACLE="obstacle"

# Function to check if service exists
check_service_exists() {
    local service=$1
    if ! systemctl list-unit-files | grep -q "^${service}.service"; then
        echo -e "${RED}Warning: Service '${service}' not found!${NC}"
        return 1
    fi
    return 0
}

# Function to get service status (enabled/disabled)
get_service_enabled_status() {
    local service=$1
    if systemctl is-enabled "$service" >/dev/null 2>&1; then
        echo "enabled"
    else
        echo "disabled"
    fi
}

# Function to get service active status (running/stopped)
get_service_active_status() {
    local service=$1
    if systemctl is-active "$service" >/dev/null 2>&1; then
        echo "running"
    else
        echo "stopped"
    fi
}

# Function to get GUI status (whether graphical.target is the boot default)
get_gui_status() {
    if [ "$(systemctl get-default)" = "graphical.target" ]; then
        echo "enabled"
    else
        echo "disabled"
    fi
}

# Function to get VNC status
get_vnc_status() {
    if systemctl is-active wayvnc.service >/dev/null 2>&1 || systemctl is-active vncserver-x11-serviced.service >/dev/null 2>&1; then
        echo "enabled"
    elif [ "$(sudo raspi-config nonint get_vnc 2>/dev/null | tail -n1)" = "0" ]; then
        echo "enabled"
    else
        echo "disabled"
    fi
}

# Function to display service status
display_service_status() {
    echo -e "\n${CYAN}=== Current Service Status ===${NC}"

    for service in "$SERVICE_OPEN" "$SERVICE_OBSTACLE"; do
        if check_service_exists "$service"; then
            local enabled_status=$(get_service_enabled_status "$service")
            local active_status=$(get_service_active_status "$service")

            # Color coding
            local enabled_color=$GREEN
            local active_color=$GREEN

            if [ "$enabled_status" = "disabled" ]; then
                enabled_color=$RED
            fi

            if [ "$active_status" = "stopped" ]; then
                active_color=$RED
            fi

            echo -e "${BLUE}Service: ${service}${NC}"
            echo -e "  Enabled: ${enabled_color}${enabled_status}${NC}"
            echo -e "  Status:  ${active_color}${active_status}${NC}"
        fi
    done

    local gui_status=$(get_gui_status)
    local vnc_status=$(get_vnc_status)
    local gui_color=$GREEN
    local vnc_color=$GREEN
    [ "$gui_status" = "disabled" ] && gui_color=$RED
    [ "$vnc_status" = "disabled" ] && vnc_color=$RED

    echo -e "${BLUE}GUI (boot default):${NC} ${gui_color}${gui_status}${NC}"
    echo -e "${BLUE}VNC:${NC}               ${vnc_color}${vnc_status}${NC}"
}

# Function to check for warnings
check_warnings() {
    local open_enabled=$(get_service_enabled_status "$SERVICE_OPEN")
    local obstacle_enabled=$(get_service_enabled_status "$SERVICE_OBSTACLE")

    if [ "$open_enabled" = "enabled" ] && [ "$obstacle_enabled" = "enabled" ]; then
        echo -e "\n${RED}⚠️  WARNING: Both services are enabled! This may cause conflicts.${NC}"
        return 1
    fi
    return 0
}

# Function to stop and disable a service
stop_and_disable() {
    local service=$1
    echo -e "  ${YELLOW}Stopping and disabling ${service}...${NC}"

    if systemctl is-active "$service" >/dev/null 2>&1; then
        sudo systemctl stop "$service"
    fi

    if systemctl is-enabled "$service" >/dev/null 2>&1; then
        sudo systemctl disable "$service"
    fi
}

# Function to enable and optionally start a service
enable_service() {
    local service=$1
    local start_service=$2  # true/false

    echo -e "  ${GREEN}Enabling ${service}...${NC}"
    sudo systemctl enable "$service"

    if [ "$start_service" = "true" ]; then
        echo -e "  ${GREEN}Starting ${service}...${NC}"
        sudo systemctl start "$service"
    fi
}

# Function to disable GUI and VNC (current session + persist across reboot)
disable_gui_vnc() {
    echo -e "  ${YELLOW}Disabling VNC...${NC}"
    sudo raspi-config nonint do_vnc 1 >/dev/null 2>&1
    sudo systemctl stop wayvnc.service >/dev/null 2>&1
    sudo systemctl stop vncserver-x11-serviced.service >/dev/null 2>&1
    sudo systemctl disable wayvnc.service >/dev/null 2>&1
    sudo systemctl disable vncserver-x11-serviced.service >/dev/null 2>&1

    echo -e "  ${YELLOW}Disabling GUI (switching to console now + on boot)...${NC}"
    sudo systemctl set-default multi-user.target >/dev/null 2>&1
    sudo systemctl isolate multi-user.target
}

# Function to enable GUI and VNC (current session + persist across reboot)
enable_gui_vnc() {
    echo -e "  ${GREEN}Enabling GUI (switching to desktop now + on boot)...${NC}"
    sudo systemctl set-default graphical.target >/dev/null 2>&1
    sudo systemctl isolate graphical.target

    echo -e "  ${GREEN}Waiting for desktop session to start...${NC}"
    for i in {1..20}; do
        if pgrep -x labwc >/dev/null 2>&1 || pgrep -x Xorg >/dev/null 2>&1; then
            sleep 1
            break
        fi
        sleep 0.5
    done

    echo -e "  ${GREEN}Enabling VNC...${NC}"
    if pgrep -x labwc >/dev/null 2>&1 || grep -q "labwc" /etc/lightdm/lightdm.conf 2>/dev/null; then
        sudo systemctl stop vncserver-x11-serviced.service >/dev/null 2>&1
        sudo systemctl disable vncserver-x11-serviced.service >/dev/null 2>&1
        sudo raspi-config nonint do_vnc 0 >/dev/null 2>&1
        sudo systemctl enable wayvnc.service >/dev/null 2>&1
        sudo systemctl restart wayvnc.service >/dev/null 2>&1
    else
        sudo raspi-config nonint do_vnc 0 >/dev/null 2>&1
    fi
}

# Function to toggle GUI+VNC based on current state
handle_toggle_gui_vnc() {
    local gui_status=$(get_gui_status)

    echo -e "\n${BLUE}Configuring GUI & VNC...${NC}"
    if [ "$gui_status" = "enabled" ]; then
        disable_gui_vnc
        echo -e "${GREEN}✓ GUI & VNC disabled${NC}"
    else
        enable_gui_vnc
        echo -e "${GREEN}✓ GUI & VNC enabled${NC}"
    fi
}

# Function to handle practice mode
handle_practice() {
    echo -e "\n${BLUE}Configuring for Practice Mode...${NC}"
    echo "  - Both services will be stopped and disabled"

    stop_and_disable "$SERVICE_OPEN"
    stop_and_disable "$SERVICE_OBSTACLE"

    echo -e "${GREEN}✓ Practice mode configured${NC}"
}

# Function to handle open mode
handle_open() {
    echo -e "\n${BLUE}Configuring for Open Mode...${NC}"
    echo "  - Stopping both services"
    echo "  - Disabling obstacle service"
    echo "  - Enabling open service"

    # Stop both services first
    if systemctl is-active "$SERVICE_OPEN" >/dev/null 2>&1; then
        sudo systemctl stop "$SERVICE_OPEN"
    fi
    if systemctl is-active "$SERVICE_OBSTACLE" >/dev/null 2>&1; then
        sudo systemctl stop "$SERVICE_OBSTACLE"
    fi

    # Disable obstacle, enable open
    stop_and_disable "$SERVICE_OBSTACLE"
    enable_service "$SERVICE_OPEN" true

    # Free up resources for the run: turn off GUI/VNC
    disable_gui_vnc

    echo -e "${GREEN}✓ Open mode configured${NC}"
}

# Function to handle obstacle mode
handle_obstacle() {
    echo -e "\n${BLUE}Configuring for Obstacle Mode...${NC}"
    echo "  - Stopping both services"
    echo "  - Disabling open service"
    echo "  - Enabling obstacle service"

    # Stop both services first
    if systemctl is-active "$SERVICE_OPEN" >/dev/null 2>&1; then
        sudo systemctl stop "$SERVICE_OPEN"
    fi
    if systemctl is-active "$SERVICE_OBSTACLE" >/dev/null 2>&1; then
        sudo systemctl stop "$SERVICE_OBSTACLE"
    fi

    # Disable open, enable obstacle
    stop_and_disable "$SERVICE_OPEN"
    enable_service "$SERVICE_OBSTACLE" true

    # Free up resources for the run: turn off GUI/VNC
    disable_gui_vnc

    echo -e "${GREEN}✓ Obstacle mode configured${NC}"
}

# Function to display menu and get user choice
show_menu() {
    echo -e "\n${CYAN}=== Robot Mode Selection ===${NC}"
    echo -e "${YELLOW}1)${NC} Practice Mode  - Both services stopped and disabled"
    echo -e "${YELLOW}2)${NC} Open Mode      - Only 'open' service started and enabled, GUI/VNC off"
    echo -e "${YELLOW}3)${NC} Obstacle Mode  - Only 'obstacle' service started and enabled, GUI/VNC off"
    echo -e "${YELLOW}4)${NC} Toggle GUI/VNC - $([ "$(get_gui_status)" = "enabled" ] && echo "currently ON, will turn OFF" || echo "currently OFF, will turn ON")"
    echo -e "${YELLOW}5)${NC} Exit"
    echo -e "\n${BLUE}Enter your choice (1-5): ${NC}"
}

# Main function
main() {
    echo -e "${CYAN}╔════════════════════════════════╗${NC}"
    echo -e "${CYAN}║     Robot Service Manager      ║${NC}"
    echo -e "${CYAN}║      Raspberry Pi 5            ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════╝${NC}"

    # Check if services exist
    services_exist=true
    for service in "$SERVICE_OPEN" "$SERVICE_OBSTACLE"; do
        if ! check_service_exists "$service"; then
            services_exist=false
        fi
    done

    if [ "$services_exist" = false ]; then
        echo -e "${RED}One or more services not found. Please check your service names.${NC}"
        exit 1
    fi

    # Show current status
    display_service_status

    # Check for warnings
    check_warnings

    # Show menu and handle user input
    while true; do
        show_menu
        read -r choice

        case $choice in
            1)
                handle_practice
                break
                ;;
            2)
                handle_open
                break
                ;;
            3)
                handle_obstacle
                break
                ;;
            4)
                handle_toggle_gui_vnc
                break
                ;;
            5)
                echo -e "${YELLOW}Exiting...${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}Invalid choice. Please enter 1-5.${NC}"
                ;;
        esac
    done

    # Show final status
    echo -e "\n${GREEN}Configuration complete!${NC}"
    display_service_status

}

# Check if running as root for systemctl commands
if [ "$EUID" -eq 0 ]; then
    echo -e "${YELLOW}Warning: Running as root. Consider running as regular user with sudo access.${NC}"
fi

# Run main function
main

