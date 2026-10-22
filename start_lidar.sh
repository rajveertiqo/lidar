#!/bin/bash
#
#   ./start_lidar.sh                run in this terminal (closing it, or losing SSH, stops guidance)
#   ./start_lidar.sh --background   run detached under systemd: it keeps running when the
#                                   terminal closes or SSH drops, and still does NOT start at boot

set -e

MODE=foreground
case "${1:-}" in
    "") ;;
    -b|--background) MODE=background ;;
    -h|--help) sed -n '2,5p' "$0" | sed 's/^#//'; exit 0 ;;
    *) echo "Unknown option: $1 (try --help)"; exit 2 ;;
esac

if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' NC=''
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ENV_FILE="$HOME/.config/shappe-guide.env"
# As the app will see them: the env file's (the installed unit's EnvironmentFile; sourced
# below as well), else this shell's, else the defaults.
CALLER_SET="${SERVER_HOST:-}${SERVER_PORT:-}"
IFS='|' read -r HOST PORT < <(
    [ -f "$ENV_FILE" ] && set -a && . "$ENV_FILE" >/dev/null 2>&1
    printf '%s|%s\n' "${SERVER_HOST:-0.0.0.0}" "${SERVER_PORT:-8000}") || true
HOST=${HOST:-0.0.0.0}
PORT=${PORT:-8000}

# A refusal below exits with this, and the units do not restart on it
# (RestartPreventExitStatus), rather than retrying every 3 s and taking the hardware the
# moment the other copy stops.
REFUSED=75
LOCK="${XDG_RUNTIME_DIR:-/tmp}/shappe-guide.lock"

# A second copy would take the same LiDAR, radars and cameras: its radar probe stops the
# boards the first is streaming from, and it stops the LiDAR motor on its way out when it
# finds the port taken (2026-10-21). So refuse, before anything touches the hardware.
already_running() {
    local pids p
    # The app as launched (".../venv/bin/uvicorn main:app"), not any command line that
    # merely mentions it, such as a grep for it.
    pids="$(pgrep -u "$(id -u)" -f '/bin/uvicorn main:app( |$)' || true)"
    if [ -n "$pids" ]; then
        echo -e "${RED}Shappe Guide is already running — not starting a second copy:${NC}"
        for p in $pids; do
            echo "  PID $p in $(readlink "/proc/$p/cwd" 2>/dev/null || echo '?')"
        done
    elif ss -Hltn "sport = :$PORT" 2>/dev/null | grep -q .; then
        echo -e "${RED}Port $PORT is already in use by another program — not starting.${NC}"
    else
        return 1
    fi
    echo -e "${YELLOW}  Stop it first: Ctrl+C in the terminal running it, or${NC}"
    echo -e "${YELLOW}    systemctl --user stop shappe-guide shappe-guide-manual${NC}"
    return 0
}

starting_elsewhere() {
    echo -e "${RED}Another ./start_lidar.sh is starting up right now — not starting a second copy.${NC}"
}

if [ "$MODE" = background ]; then
    if already_running; then exit $REFUSED; fi
    if ! flock -n "$LOCK" true; then starting_elsewhere; exit $REFUSED; fi
    # The installed unit when it runs this checkout (watchdog, restart, its env file);
    # otherwise a one-off unit with the same safety net, which can also carry a port or
    # host set in this shell. Neither is enabled, so neither starts at boot.
    if [ -z "$CALLER_SET" ] && [ "$(systemctl --user show -P WorkingDirectory shappe-guide.service 2>/dev/null)" = "$SCRIPT_DIR" ]; then
        UNIT=shappe-guide.service
        systemctl --user reset-failed "$UNIT" 2>/dev/null || true
        systemctl --user start --no-block "$UNIT"
    else
        UNIT=shappe-guide-manual.service
        systemctl --user reset-failed "$UNIT" 2>/dev/null || true
        systemd-run --user --no-block --collect --unit="$UNIT" --working-directory="$SCRIPT_DIR" \
            -E SERVER_HOST="$HOST" -E SERVER_PORT="$PORT" \
            -p Type=notify -p NotifyAccess=main -p WatchdogSec=30 -p TimeoutStartSec=180 \
            -p TimeoutStopSec=60 -p TimeoutAbortSec=10 -p LimitCORE=0 \
            -p Restart=on-failure -p RestartSec=3 -p RestartPreventExitStatus=$REFUSED \
            "$SCRIPT_DIR/start_lidar.sh"
    fi
    echo -e "${GREEN}Started in the background as $UNIT.${NC}"
    echo "  It keeps running when this terminal closes or SSH drops; it does NOT start at boot."
    echo "  follow it:  journalctl --user-unit=$UNIT -f"
    echo "  stop it:    systemctl --user stop $UNIT"
    if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
        echo -e "${YELLOW}  Note: lingering is off for $USER, so it stops when you log out of every${NC}"
        echo -e "${YELLOW}  session. To keep it running: sudo loginctl enable-linger $USER${NC}"
        echo -e "${YELLOW}  (that starts nothing at boot by itself; no Shappe unit is enabled).${NC}"
    fi
    exit 0
fi

exec 9>"$LOCK"
if ! flock -n 9; then
    # A running app keeps the lock (below), so say which, if it is that.
    already_running || starting_elsewhere
    exit $REFUSED
fi
# fd 9 stays open into uvicorn below (exec keeps it), so the lock lasts as long as the app.
if already_running; then exit $REFUSED; fi

# Line by line into the journal, not in 8 KB blocks: the last lines before a crash count.
export PYTHONUNBUFFERED=1

export HAILORT_LOGGER_PATH=NONE

export HAILO_MONITOR=1

export LIBCAMERA_LOG_LEVELS="${LIBCAMERA_LOG_LEVELS:-*:ERROR}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  LiDAR System Startup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

PRESET_TOKEN="${SHAPPE_ADMIN_TOKEN:-}"
# A line that sets it to something (a commented or empty one does not count).
TOKEN_LINE_RE="^[[:space:]]*(export[[:space:]]+)?SHAPPE_ADMIN_TOKEN=[\"']?[^[:space:]\"'#]"
if [ -z "$PRESET_TOKEN" ] && ! { [ -f "$ENV_FILE" ] && grep -Eq "$TOKEN_LINE_RE" "$ENV_FILE"; }; then
    TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(12))' 2>/dev/null || true)"
    SEP=""
    if [ -s "$ENV_FILE" ] && [ -n "$(tail -c 1 "$ENV_FILE" 2>/dev/null)" ]; then SEP=$'\n'; fi
    if [ -n "$TOKEN" ] && mkdir -p "$(dirname "$ENV_FILE")" 2>/dev/null \
            && (umask 077 && printf '%sSHAPPE_ADMIN_TOKEN=%s\n' "$SEP" "$TOKEN" >> "$ENV_FILE" \
                && chmod 600 "$ENV_FILE") 2>/dev/null; then
        if [ -t 1 ]; then
            echo -e "${GREEN}Created an admin token: ${TOKEN}${NC}"
        else
            echo -e "${GREEN}Created an admin token (not shown: this output is not a terminal)${NC}"
        fi
        echo -e "${BLUE}  stored in $ENV_FILE — the test pages ask for it once per browser${NC}"
    else
        echo -e "${YELLOW}Warning: could not write an admin token to $ENV_FILE —${NC}"
        echo -e "${YELLOW}  radar calibration, radar restart and the stride will be refused${NC}"
    fi
    unset TOKEN SEP
    echo ""
fi
if [ -f "$ENV_FILE" ]; then
    set -a
    . "$ENV_FILE"
    set +a
fi
if [ -n "$PRESET_TOKEN" ]; then
    export SHAPPE_ADMIN_TOKEN="$PRESET_TOKEN"
fi
if [ -n "${SHAPPE_ADMIN_TOKEN:-}" ]; then
    echo -e "${BLUE}Admin token loaded (see $ENV_FILE)${NC}"
    echo ""
fi

if [ ! -d "venv" ]; then
    echo -e "${RED}Error: Virtual environment not found!${NC}"
    echo -e "${YELLOW}Please run setup first:${NC}"
    echo "  python3 -m venv --system-site-packages venv"
    echo "  source venv/bin/activate"
    echo "  pip install -r requirements.txt"
    exit 1
fi

echo -e "${GREEN}[1/6] Activating virtual environment...${NC}"
source venv/bin/activate

if [ ! -f "main.py" ]; then
    echo -e "${RED}Error: main.py not found!${NC}"
    exit 1
fi

echo -e "${GREEN}[2/6] Checking LiDAR device...${NC}"
LIDAR_DEV="$(python3 -c 'import os, config; print(next((p for p in config.LIDAR_PORT_CANDIDATES if os.path.exists(p)), config.LIDAR_PORT))' 2>/dev/null || echo /dev/ttyUSB0)"
if [ ! -e "$LIDAR_DEV" ]; then
    echo -e "${YELLOW}Warning: LiDAR device $LIDAR_DEV not found${NC}"
    echo -e "${YELLOW}System will start but LiDAR may not function${NC}"
else
    echo -e "${BLUE}  LiDAR device detected: $LIDAR_DEV -> $(readlink -f "$LIDAR_DEV")${NC}"
fi

echo -e "${GREEN}[3/6] Checking radar array...${NC}"
RADAR_ERR=$(mktemp)
RADAR_OUT=$(python -m utils.radar_utils --check 2>"$RADAR_ERR") && RADAR_RC=0 || RADAR_RC=$?
echo -e "${BLUE}${RADAR_OUT}${NC}"

if [ "$RADAR_RC" -eq 4 ] || grep -q '^Traceback' "$RADAR_ERR"; then
    echo -e "${RED}Error: the radar check itself failed (software error, not a cable)${NC}"
    sed 's/^/  /' "$RADAR_ERR"
    RADAR_RC=0
fi
rm -f "$RADAR_ERR"

if [ $(( RADAR_RC & 1 )) -ne 0 ]; then
    echo -e "${YELLOW}Warning: a radar board did not answer on any USB port${NC}"
    echo -e "${YELLOW}  Check its cable and hub socket. A board that answers nothing${NC}"
    echo -e "${YELLOW}  has hung its firmware — unplug and replug its USB-C cable.${NC}"
fi
if [ $(( RADAR_RC & 2 )) -ne 0 ]; then
    echo -e "${YELLOW}Warning: radar board(s) have NO MEASURED BEARING and will not steer${NC}"
    echo -e "${YELLOW}  This is NOT a cable fault — those boards stream fine. Until a${NC}"
    echo -e "${YELLOW}  bearing is measured they feed no direction, so GLASS IS NOT${NC}"
    echo -e "${YELLOW}  DETECTED on those arcs and the LiDAR is walking blind to it.${NC}"
    echo -e "${YELLOW}  Fix: open http://<pi>:${SERVER_PORT:-8000}/radar/test once the server${NC}"
    echo -e "${YELLOW}  is up, hold a hand ~1.5 ft in front of ONE board at a time, and${NC}"
    echo -e "${YELLOW}  set the bearing of whichever row's distance drops.${NC}"
fi

echo -e "${GREEN}[4/6] Checking Hailo NPU...${NC}"
if command -v hailortcli >/dev/null 2>&1; then
    HAILO_OUT=$(timeout 15 hailortcli scan 2>&1 | head -4 || true)
    if echo "$HAILO_OUT" | grep -qi "not found"; then
        echo -e "${YELLOW}  No Hailo device found — object detection and segmentation will be OFF${NC}"
        echo -e "${YELLOW}  (check the HAT seating, or a previous run still holding the device)${NC}"
    else
        echo -e "${BLUE}$(echo "$HAILO_OUT" | sed 's/^/  /')${NC}"
    fi
else
    echo -e "${YELLOW}  hailortcli not installed — cannot verify the NPU${NC}"
fi

echo -e "${GREEN}[5/6] Checking dependencies...${NC}"
if ! python -c "import fastapi, uvicorn" 2>/dev/null; then
    echo -e "${RED}Error: Required packages not installed!${NC}"
    echo -e "${YELLOW}Please install dependencies:${NC}"
    echo "  pip install -r requirements.txt"
    exit 1
fi

echo -e "${GREEN}[6/6] Starting FastAPI server...${NC}"
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Server Starting...${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
if [ -t 0 ]; then
    echo -e "${YELLOW}Closing this terminal or losing SSH stops guidance. For a walk, start it${NC}"
    echo -e "${YELLOW}with ./start_lidar.sh --background instead.${NC}"
    echo ""
fi

GRACE="$(python3 -c 'import config; print(config.SERVER_GRACEFUL_SHUTDOWN_S)' 2>/dev/null || echo 5)"
exec uvicorn main:app --host "$HOST" --port "$PORT" --no-access-log --log-level warning --timeout-graceful-shutdown "${GRACE%.*}"
