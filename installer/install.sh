#!/usr/bin/env bash
set -Eeuo pipefail

REPO_URL="https://github.com/AloeVeraZ/OmniBot.git"
APP_DIR="${OMNIBOT_APP_DIR:-$HOME/OmniBot}"
RUNNER="$APP_DIR/run_omnibot.sh"
LOG_FILE="$APP_DIR/omnibot.log"

say() { printf '\n\033[1;36m[OmniBot]\033[0m %s\n' "$*"; }
fail() { printf '\n\033[1;31m[OmniBot ERROR]\033[0m %s\n' "$*" >&2; exit 1; }
trap 'fail "Installation stopped on line $LINENO. Fix the error above and rerun the installer."' ERR

if [ "$(id -u)" -eq 0 ]; then
    fail "Run this as the normal Raspberry Pi user, without sudo."
fi
command -v sudo >/dev/null 2>&1 || fail "sudo is required."

apt_get() {
    sudo env DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=120 "$@"
}

say "Installing Raspberry Pi, pygame, GPIO, and Bluetooth packages..."
apt_get update

GPIO_PACKAGE="python3-rpi.gpio"
if apt-cache show python3-rpi-lgpio >/dev/null 2>&1; then
    GPIO_PACKAGE="python3-rpi-lgpio"
fi

apt_get install -y \
    ca-certificates \
    git \
    python3 \
    python3-pygame \
    "$GPIO_PACKAGE" \
    bluez

if ! command -v labwc >/dev/null 2>&1 && \
   ! command -v startlxde-pi >/dev/null 2>&1; then
    say "No graphical desktop detected. Installing the Raspberry Pi desktop..."
    if apt-cache show rpd-wayland-core >/dev/null 2>&1; then
        apt_get install -y rpd-wayland-core rpd-theme rpd-preferences lightdm
    elif apt-cache show raspberrypi-ui-mods >/dev/null 2>&1; then
        apt_get install -y raspberrypi-ui-mods lightdm
    else
        fail "Desktop packages were not found. Flash Raspberry Pi OS with Desktop and rerun."
    fi
fi

sudo systemctl set-default graphical.target
sudo systemctl enable lightdm 2>/dev/null || true

say "Downloading OmniBot..."
install_fresh_copy() {
    local reason="$1"
    local stamp="$(date +%Y%m%d-%H%M%S).$$"
    local backup="${APP_DIR}.backup.${stamp}"
    local fresh="${APP_DIR}.installing.${stamp}"
    say "$reason"
    git clone --branch main --single-branch "$REPO_URL" "$fresh"
    mv "$APP_DIR" "$backup"
    mv "$fresh" "$APP_DIR"
    say "The previous folder was preserved at $backup"
}

if [ -d "$APP_DIR/.git" ]; then
    checkout_valid=true
    changes=""
    if changes="$(git -C "$APP_DIR" status --porcelain --untracked-files=all)"; then
        changes="$(printf '%s\n' "$changes" | grep -vFx '?? run_omnibot.sh' | grep -vFx '?? omnibot.log' || true)"
    else
        checkout_valid=false
    fi

    if [ "$checkout_valid" != true ]; then
        install_fresh_copy "The existing Git checkout is damaged; installing a clean copy."
    elif ! git -C "$APP_DIR" fetch --prune origin main; then
        install_fresh_copy "The existing checkout could not be updated; installing a clean copy."
    elif [ -n "$changes" ]; then
        install_fresh_copy "Local changes were found; installing a clean copy."
    elif ! git -C "$APP_DIR" show-ref --verify --quiet refs/heads/main; then
        install_fresh_copy "The existing checkout has no main branch; installing a clean copy."
    elif ! git -C "$APP_DIR" merge-base --is-ancestor main origin/main; then
        install_fresh_copy "Local commits were found; installing a clean copy."
    else
        git -C "$APP_DIR" checkout -f main
        git -C "$APP_DIR" reset --hard origin/main
    fi
elif [ -e "$APP_DIR" ]; then
    install_fresh_copy "A non-Git OmniBot folder was found; installing a clean copy."
else
    git clone --branch main --single-branch "$REPO_URL" "$APP_DIR"
fi

python3 -m py_compile "$APP_DIR/omni_kinematics.py" "$APP_DIR/omni_robot.py"
python3 -c 'import pygame; import RPi.GPIO; print("pygame and GPIO imports passed.")'
PYTHONPATH="$APP_DIR" python3 -m unittest discover -s "$APP_DIR/tests" -v

say "Creating the launcher and desktop auto-start..."
cat > "$RUNNER" <<EOF
#!/usr/bin/env bash
set -u
cd "$APP_DIR"
exec 9>"$APP_DIR/.omnibot.lock"
flock -n 9 || exit 0
printf '\n===== OmniBot startup: %s =====\n' "\$(date)" >> "$LOG_FILE"
exec python3 "$APP_DIR/omni_robot.py" >> "$LOG_FILE" 2>&1
EOF
chmod +x "$RUNNER"

mkdir -p "$HOME/.config/autostart" "$HOME/.config/labwc"
cat > "$HOME/.config/autostart/omnibot.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=OmniBot
Comment=Three-wheel omni robot controller
Exec=$RUNNER
Path=$APP_DIR
Terminal=false
StartupNotify=false
X-GNOME-Autostart-enabled=true
EOF

touch "$HOME/.config/labwc/autostart"
sed -i '/# OMNIBOT START/,/# OMNIBOT END/d' "$HOME/.config/labwc/autostart"
cat >> "$HOME/.config/labwc/autostart" <<EOF
# OMNIBOT START
$RUNNER &
# OMNIBOT END
EOF

if command -v raspi-config >/dev/null 2>&1; then
    sudo raspi-config nonint do_boot_behaviour B4 || true
fi
sudo systemctl enable bluetooth 2>/dev/null || true

say "Installation complete. The Pi will reboot in five seconds."
echo "Your existing Bluetooth pairing is preserved."
echo "Runtime log: $LOG_FILE"
sync
sleep 5
sudo reboot
