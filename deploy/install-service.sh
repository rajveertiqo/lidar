#!/bin/bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIT=shappe-guide.service
UNIT_DIR="$HOME/.config/systemd/user"
ENV_FILE="$HOME/.config/shappe-guide.env"

if [ "${1:-}" = "--remove" ]; then
    systemctl --user disable --now "$UNIT" 2>/dev/null || true
    rm -f "$UNIT_DIR/$UNIT"
    systemctl --user daemon-reload
    echo "Removed $UNIT (kept $ENV_FILE)."
    exit 0
fi

for g in dialout spi video audio; do
    if ! id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
        echo "Warning: $USER is not in group '$g' — the service will not reach that hardware."
        echo "         sudo adduser $USER $g   (then log out and back in)"
    fi
done

mkdir -p "$UNIT_DIR"
sed "s|@APP_DIR@|$APP_DIR|g" "$APP_DIR/deploy/$UNIT" > "$UNIT_DIR/$UNIT"
echo "Installed $UNIT_DIR/$UNIT (app in $APP_DIR)"

TOKEN_LINE_RE="^[[:space:]]*(export[[:space:]]+)?SHAPPE_ADMIN_TOKEN=[\"']?[^[:space:]\"'#]"
if [ -f "$ENV_FILE" ] && grep -Eq "$TOKEN_LINE_RE" "$ENV_FILE"; then
    echo "Keeping the existing admin token in $ENV_FILE"
else
    TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(12))')"
    SEP=""
    if [ -s "$ENV_FILE" ] && [ -n "$(tail -c 1 "$ENV_FILE")" ]; then SEP=$'\n'; fi
    (umask 077; printf '%sSHAPPE_ADMIN_TOKEN=%s\n' "$SEP" "$TOKEN" >> "$ENV_FILE")
    chmod 600 "$ENV_FILE"
    echo
    if [ -t 1 ]; then
        echo "Admin token (radar calibration, radar restart, stride): $TOKEN"
    else
        echo "Created an admin token (radar calibration, radar restart, stride)."
        echo "  Not shown: this output is not a terminal. Read it with"
        echo "  grep SHAPPE_ADMIN_TOKEN $ENV_FILE"
    fi
    echo "  stored in $ENV_FILE — the test pages ask for it once per browser."
    unset TOKEN SEP
fi

systemctl --user daemon-reload
systemctl --user enable "$UNIT"

if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
    echo
    echo "Enabling lingering so the service starts at boot (needs sudo):"
    sudo loginctl enable-linger "$USER"
fi

cat <<EOF

Done. The guide now starts at boot. Next:
  systemctl --user start shappe-guide        start it now (stop any ./start_lidar.sh first)
  journalctl --user-unit=shappe-guide -f     follow its log
  systemctl --user status shappe-guide       is it running, and since when
  curl -s localhost:8000/health              ok / degraded / starting / down
EOF
