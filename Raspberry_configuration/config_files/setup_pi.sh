#!/bin/bash

# Sprawdzenie, czy skrypt został uruchomiony z uprawnieniami root
if [ "$EUID" -ne 0 ]; then
  echo "Proszę uruchomić skrypt jako root (np. sudo ./setup_pi.sh)"
  exit 1
fi

# Pobranie bezwzględnej ścieżki katalogu repozytorium
SCRIPT_DIR=$(dirname "$(realpath "$0")")


echo "--- 1. Konfiguracja parametrów startowych kernela ---"
if [ -f "$SCRIPT_DIR/cmdline.txt" ]; then
    cp "$SCRIPT_DIR/cmdline.txt" /boot/firmware/cmdline.txt
    echo "Plik cmdline.txt nadpisany."
fi

echo "--- 2. Konfiguracja sprzętowa ---"
if [ -f "$SCRIPT_DIR/config.txt" ]; then
    cp "$SCRIPT_DIR/config.txt" /boot/firmware/config.txt
    echo "Plik config.txt nadpisany."
fi

echo "--- 3. Instalacja wymaganych pakietów ---"
if [ -f "$SCRIPT_DIR/manual_packages.txt" ]; then
    apt-get update
    PACKAGES=$(tr '\n' ' ' < "$SCRIPT_DIR/apt_packages.txt")
    # Dodano pakiety python3-venv i python3-pip niezbędne do utworzenia środowiska wirtualnego
    apt-get install -y $PACKAGES
    echo "Pakiety zainstalowane."
    sudo apt-get remove modemmanager -y
    echo "Odinstalowano modemanager"
else
    echo "Błąd: Brak pliku apt_packages.txt"
fi

echo "--- 4. Konfiguracja grup użytkownika ---"
if [ -f "$SCRIPT_DIR/user_groups.txt" ]; then
    USER_LINE=$(cat "$SCRIPT_DIR/user_groups.txt")
    USERNAME=$(echo "$USER_LINE" | awk -F' : ' '{print $1}' | tr -d ' ')
    GROUPS=$(echo "$USER_LINE" | awk -F' : ' '{print $2}' | tr ' ' ',')
    
    if ! id -u "$USERNAME" > /dev/null 2>&1; then
        useradd -m -s /bin/bash "$USERNAME"
        echo "Utworzono nowego użytkownika: $USERNAME"
    fi
    usermod -aG "$GROUPS" "$USERNAME"
    echo "Grupy zostały zaktualizowane."
else
    echo "Błąd: Brak pliku user_groups.txt"
fi




USER_HOME="/home/$USERNAME"
echo "Tworzenie wirtualnego środowiska w $USER_HOME/Mavlink..."
# Wykonanie komend w imieniu użytkownika (np. pi5), a nie jako root!
sudo -u "$USERNAME" bash -c "
    mkdir -p $USER_HOME/Mavlink
    cd $USER_HOME/Mavlink
    python3 -m venv venvMavlink
    chmod u+x ./venvMavlink/bin/activate
    source ./venvMavlink/bin/activate
    echo "aktywowano środowisko wirtualne"
    pip install -r '$SCRIPT_DIR/requirements.txt'
"

echo "--- ZAKOŃCZONO ---"
echo "Zależności Pythona zostały zainstalowane. Aby zastosować wszystkie zmiany, wykonaj:"
echo "sudo reboot"

