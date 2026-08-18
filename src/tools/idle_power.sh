#!/usr/bin/env bash
# Idle-power trim for the Greenbotics Pi 5.
#
# Cuts the current the robot draws while it sits BETWEEN runs (SSH session open,
# nothing driving). Everything it does is reversible and nothing touches the
# robot's own hardware (motors/servo/ToF/camera) -- those are already powered
# down by each program's cleanup path when it exits.
#
#   sudo ./idle_power.sh apply    # enter low-power idle
#   sudo ./idle_power.sh revert   # restore full performance
#   ./idle_power.sh status        # show governor / clock / power right now
#   sudo ./idle_power.sh setup    # PERSISTENT Pi-config savings (reboot to apply)
#   sudo ./idle_power.sh unsetup  # undo setup (reboot to apply)
#
# `apply` switches the CPU governor to powersave, which pins the ARM clock to
# its minimum -- a run started like that would miss its 17.7 ms frame budget.
# You do NOT have to remember to revert: both main programs call
# src/tools/power_mode.ensure_performance() at startup, which reverts the trim
# automatically before anything latency-sensitive starts.
#
# What apply does (each step skipped silently if the knob doesn't exist):
#   1. CPU governor -> powersave           (~0.4-0.8 W at idle on a Pi 5)
#   2. Wi-Fi power management -> on        (~0.2-0.4 W; SSH stays up, adds a few
#                                           ms of latency to the first packet)
#   3. Bluetooth -> rfkill blocked         (~0.05 W; skip with KEEP_BT=1)
#   4. Ethernet PHY -> down if unplugged   (~0.2-0.5 W; only when no carrier)
#   5. ACT + PWR LEDs -> off               (~0.02 W)
# Previous values are saved to $STATE so revert restores exactly what was there.
#
# What setup does (survives reboots; each is inert until the next reboot):
#   a. config.txt: marker-delimited block -- onboard ACT/PWR LEDs off at boot,
#      Bluetooth controller disabled (dtoverlay=disable-bt; it is unused and
#      rfkill-blocked already), audio block off (snd driver unused). No clock
#      changes -- nothing in setup may ever slow a run down.
#   b. EEPROM: POWER_OFF_ON_HALT=1 -- after `sudo halt` the board draws ~0.01 W
#      instead of ~1.3 W, which is what actually saves the battery when the
#      robot is "off" between sessions. The power button still wakes it.
#   A timestamped backup of config.txt is kept next to it before any edit.

set -u

STATE="${STATE:-/tmp/greenbotics_idle_power.state}"
WLAN="${WLAN:-wlan0}"
ETH="${ETH:-eth0}"
KEEP_BT="${KEEP_BT:-0}"

say() { echo "[idle_power] $*"; }

need_root() {
    if [ "$(id -u)" -ne 0 ]; then
        say "ERROR: '$1' needs root. Re-run with sudo."
        exit 1
    fi
}

current_governor() {
    cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor 2>/dev/null || echo "?"
}

led_trigger() {  # $1 = led name; prints the active trigger (the [bracketed] one)
    tr ' ' '\n' < "/sys/class/leds/$1/trigger" 2>/dev/null \
        | grep -m1 '^\[' | tr -d '[]'
}

pmic_total_watts() {
    vcgencmd pmic_read_adc 2>/dev/null | awk '
        NF >= 2 {
            base = $1; sub(/_[AV]$/, "", base)
            if (match($2, /=[-0-9.]+A/)) c[base] = substr($2, RSTART + 1, RLENGTH - 2) + 0
            else if (match($2, /=[-0-9.]+V/)) v[base] = substr($2, RSTART + 1, RLENGTH - 2) + 0
        }
        END { t = 0; for (r in c) if (r in v) t += c[r] * v[r]; if (t > 0) printf "%.2f", t }'
}

apply() {
    need_root apply
    : > "$STATE"

    # 1. CPU governor
    gov=$(current_governor)
    echo "governor=$gov" >> "$STATE"
    for pol in /sys/devices/system/cpu/cpufreq/policy*; do
        echo powersave > "$pol/scaling_governor" 2>/dev/null || true
    done
    say "cpu governor: $gov -> $(current_governor)"

    # 2. Wi-Fi power save (keeps the link up)
    if command -v iw >/dev/null && iw dev "$WLAN" info >/dev/null 2>&1; then
        ps=$(iw dev "$WLAN" get power_save 2>/dev/null | awk '{print $NF}')
        echo "wifi_ps=$ps" >> "$STATE"
        iw dev "$WLAN" set power_save on 2>/dev/null \
            && say "wifi power_save: $ps -> on" || say "wifi power_save: could not set"
    fi

    # 3. Bluetooth
    if [ "$KEEP_BT" != "1" ] && command -v rfkill >/dev/null; then
        if rfkill list bluetooth 2>/dev/null | grep -q "Soft blocked: no"; then
            echo "bt_was=on" >> "$STATE"
            rfkill block bluetooth && say "bluetooth: blocked"
        fi
    fi

    # 4. Ethernet -- only if nothing is plugged in
    if [ -e "/sys/class/net/$ETH" ]; then
        carrier=$(cat "/sys/class/net/$ETH/carrier" 2>/dev/null || echo 0)
        state=$(cat "/sys/class/net/$ETH/operstate" 2>/dev/null || echo unknown)
        if [ "$carrier" = "0" ] && [ "$state" != "down" ]; then
            echo "eth_was=up" >> "$STATE"
            ip link set "$ETH" down && say "$ETH: down (no carrier)"
        fi
    fi

    # 5. Onboard LEDs
    for led in ACT PWR; do
        if [ -d "/sys/class/leds/$led" ]; then
            trig=$(led_trigger "$led")
            echo "led_${led}=$trig" >> "$STATE"
            echo none > "/sys/class/leds/$led/trigger" 2>/dev/null || true
            echo 0 > "/sys/class/leds/$led/brightness" 2>/dev/null || true
            say "led $led: off (was trigger '$trig')"
        fi
    done

    say "applied. State saved to $STATE -- run 'revert' BEFORE the next robot run."
}

revert() {
    need_root revert
    if [ ! -f "$STATE" ]; then
        say "no state file at $STATE; restoring defaults instead."
    fi

    gov=$(grep -s '^governor=' "$STATE" | cut -d= -f2)
    gov=${gov:-ondemand}
    for pol in /sys/devices/system/cpu/cpufreq/policy*; do
        echo "$gov" > "$pol/scaling_governor" 2>/dev/null || true
    done
    say "cpu governor: -> $(current_governor)"

    if command -v iw >/dev/null && iw dev "$WLAN" info >/dev/null 2>&1; then
        ps=$(grep -s '^wifi_ps=' "$STATE" | cut -d= -f2)
        iw dev "$WLAN" set power_save "${ps:-off}" 2>/dev/null \
            && say "wifi power_save: -> ${ps:-off}"
    fi

    if grep -sq '^bt_was=on' "$STATE" && command -v rfkill >/dev/null; then
        rfkill unblock bluetooth && say "bluetooth: unblocked"
    fi

    if grep -sq '^eth_was=up' "$STATE" && [ -e "/sys/class/net/$ETH" ]; then
        ip link set "$ETH" up && say "$ETH: up"
    fi

    for led in ACT PWR; do
        if [ -d "/sys/class/leds/$led" ]; then
            trig=$(grep -s "^led_${led}=" "$STATE" | cut -d= -f2)
            case "$led" in ACT) def=mmc0 ;; *) def=default-on ;; esac
            echo "${trig:-$def}" > "/sys/class/leds/$led/trigger" 2>/dev/null || true
            say "led $led: trigger -> ${trig:-$def}"
        fi
    done

    rm -f "$STATE"
    say "reverted -- full performance restored."
}

status() {
    say "cpu governor : $(current_governor)"
    say "arm clock    : $(vcgencmd measure_clock arm 2>/dev/null | cut -d= -f2 | awk '{printf "%d MHz", $1/1e6}')"
    say "cpu temp     : $(vcgencmd measure_temp 2>/dev/null | cut -d= -f2)"
    say "throttled    : $(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
    if command -v iw >/dev/null && iw dev "$WLAN" info >/dev/null 2>&1; then
        say "wifi ps      : $(iw dev "$WLAN" get power_save 2>/dev/null | awk '{print $NF}')"
    fi
    [ -e "/sys/class/net/$ETH" ] && say "$ETH          : $(cat /sys/class/net/$ETH/operstate 2>/dev/null)"
    command -v rfkill >/dev/null && say "bluetooth    : $(rfkill list bluetooth 2>/dev/null | grep -m1 'Soft blocked' | awk '{print ($3=="yes") ? "blocked" : "on"}')"
    w=$(pmic_total_watts)
    [ -n "$w" ] && say "pmic total   : ${w} W (regulator-rail output, not battery input)"
    if [ -f "$STATE" ]; then
        say "state file   : $STATE (idle trim is APPLIED -- revert before a run)"
    fi
    return 0
}

BOOT_CFG="${BOOT_CFG:-/boot/firmware/config.txt}"
MARK_BEGIN="# --- greenbotics idle-power (idle_power.sh setup) ---"
MARK_END="# --- end greenbotics idle-power ---"

setup() {
    need_root setup

    # a. config.txt block (idempotent; backup kept next to the file)
    if grep -qF "$MARK_BEGIN" "$BOOT_CFG" 2>/dev/null; then
        say "config.txt: idle-power block already present"
    else
        cp "$BOOT_CFG" "${BOOT_CFG}.bak-idlepower-$(date +%Y%m%d-%H%M%S)"
        cat >> "$BOOT_CFG" <<EOF

$MARK_BEGIN
# Everything here takes effect at the NEXT BOOT only.
# Onboard status LEDs off (the robot has its own gpiozero LED for signalling).
dtparam=act_led_trigger=none
dtparam=act_led_activelow=off
dtparam=pwr_led_trigger=none
# PWR (red) is wired active-low in hardware, opposite of ACT -- "on" here
# matches that wiring so trigger=none actually goes dark instead of solid red.
dtparam=pwr_led_activelow=on
# Bluetooth controller off entirely (unused; was already rfkill-blocked).
dtoverlay=disable-bt
# Audio driver off (overrides the dtparam=audio=on above; unused on the robot).
dtparam=audio=off
# NOTE deliberately NO arm_freq_min override: lowering the 1500 MHz idle floor
# saves little (race-to-idle) and could add clock-ramp jitter mid-run. Nothing
# in this block may ever slow a run down.
$MARK_END
EOF
        say "config.txt: idle-power block appended (backup kept alongside)"
    fi

    # b. EEPROM: near-zero power after `sudo halt` (power button still wakes it)
    if command -v rpi-eeprom-config >/dev/null; then
        cur=$(rpi-eeprom-config)
        if printf '%s\n' "$cur" | grep -q '^POWER_OFF_ON_HALT=1'; then
            say "eeprom: POWER_OFF_ON_HALT already 1"
        else
            tmp=$(mktemp)
            if printf '%s\n' "$cur" | grep -q '^POWER_OFF_ON_HALT='; then
                printf '%s\n' "$cur" | sed 's/^POWER_OFF_ON_HALT=.*/POWER_OFF_ON_HALT=1/' > "$tmp"
            else
                { printf '%s\n' "$cur"; echo "POWER_OFF_ON_HALT=1"; } > "$tmp"
            fi
            if rpi-eeprom-config --apply "$tmp" >/dev/null 2>&1; then
                say "eeprom: POWER_OFF_ON_HALT=1 staged (written on next reboot)"
            else
                say "eeprom: WARNING -- rpi-eeprom-config --apply failed; halt power unchanged"
            fi
            rm -f "$tmp"
        fi
    else
        say "eeprom: rpi-eeprom-config not found; skipping halt-power change"
    fi

    say "setup done. Nothing changes until the next reboot."
}

unsetup() {
    need_root unsetup

    if grep -qF "$MARK_BEGIN" "$BOOT_CFG" 2>/dev/null; then
        cp "$BOOT_CFG" "${BOOT_CFG}.bak-idlepower-$(date +%Y%m%d-%H%M%S)"
        tmp=$(mktemp)
        awk -v b="$MARK_BEGIN" -v e="$MARK_END" \
            '$0==b {skip=1} !skip {print} skip && $0==e {skip=0}' "$BOOT_CFG" > "$tmp"
        cat "$tmp" > "$BOOT_CFG"
        rm -f "$tmp"
        say "config.txt: idle-power block removed (backup kept alongside)"
    else
        say "config.txt: no idle-power block found"
    fi

    if command -v rpi-eeprom-config >/dev/null; then
        cur=$(rpi-eeprom-config)
        if printf '%s\n' "$cur" | grep -q '^POWER_OFF_ON_HALT=1'; then
            tmp=$(mktemp)
            printf '%s\n' "$cur" | sed 's/^POWER_OFF_ON_HALT=.*/POWER_OFF_ON_HALT=0/' > "$tmp"
            rpi-eeprom-config --apply "$tmp" >/dev/null 2>&1 \
                && say "eeprom: POWER_OFF_ON_HALT=0 staged (written on next reboot)"
            rm -f "$tmp"
        else
            say "eeprom: POWER_OFF_ON_HALT already 0"
        fi
    fi

    say "unsetup done. Nothing changes until the next reboot."
}

case "${1:-}" in
    apply)   apply ;;
    revert)  revert ;;
    status)  status ;;
    setup)   setup ;;
    unsetup) unsetup ;;
    *) echo "usage: $0 apply|revert|status|setup|unsetup   (see header comments)"; exit 1 ;;
esac
