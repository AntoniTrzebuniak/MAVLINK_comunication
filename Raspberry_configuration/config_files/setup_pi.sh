#!/bin/bash

# Sprawdzenie, czy skrypt został uruchomiony z uprawnieniami root
if [ "$EUID" -ne 0 ]; then
  echo "Proszę uruchomić skrypt jako root (np. sudo ./setup_pi.sh)"
  exit 1
fi

# Pobranie bezwzględnej ścieżki katalogu repozytorium
SCRIPT_DIR=$(dirname "$(realpath "$0")")
# Pobranie samej nazwy folderu, do którego sklonowałeś repo (potrzebne do wykluczenia z usuwania)
REPO_DIR_NAME=$(basename "$SCRIPT_DIR")



echo "--- 1. Ustawianie nazwy hosta ---"
if [ -f "$SCRIPT_DIR/hostname.txt" ]; then
    NEW_HOSTNAME=$(cat "$SCRIPT_DIR/hostname.txt" | tr -d '[:space:]')
    hostnamectl set-hostname "$NEW_HOSTNAME"
    sed -i '/127.0.1.1/d' /etc/hosts
    echo "127.0.1.1 $NEW_HOSTNAME" >> /etc/hosts
    echo "Nazwa hosta zmieniona na: $NEW_HOSTNAME"
else
    echo "Błąd: Brak pliku hostname.txt"
fi

echo "--- 2. Konfiguracja parametrów startowych kernela ---"
if [ -f "$SCRIPT_DIR/cmdline.txt" ]; then
    cp "$SCRIPT_DIR/cmdline.txt" /boot/firmware/cmdline.txt
    echo "Plik cmdline.txt nadpisany."
fi

echo "--- 3. Konfiguracja sprzętowa ---"
if [ -f "$SCRIPT_DIR/config.txt" ]; then
    cp "$SCRIPT_DIR/config.txt" /boot/firmware/config.txt
    echo "Plik config.txt nadpisany."
fi

echo "--- 4. Instalacja wymaganych pakietów ---"
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

echo "--- 5. Konfiguracja grup użytkownika ---"
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

echo "--- 6. Konfiguracja sieci wifi ---"

TEMP_SSID="PapieskiWifi"
TEMP_PASSWORD="213769420"
sudo nmcli dev wifi connect "$TEMP_SSID" password "$TEMP_PASSWORD"
sudo nmcli connection modify "PapieskiWifi" connection.autoconnect-priority 10
echo "dodano sieć $TEMP_SSID z priorytetem 10"

TEMP_SSID="moje nie tylkaj"
TEMP_PASSWORD="12345678"
sudo nmcli dev wifi connect "$TEMP_SSID" password "$TEMP_PASSWORD"
sudo nmcli connection modify "$TEMP_SSID" connection.autoconnect-priority 9
echo "dodano sieć $TEMP_SSID z priorytetem 9"

unset TEMP_PASSWORD

echo "--- 6. Czyszczenie katalogu domowego i konfiguracja środowiska Mavlink ---"
USER_HOME="/home/$USERNAME"

echo "Czyszczenie folderów w $USER_HOME z wyjątkiem $REPO_DIR_NAME..."
for d in "$USER_HOME"/*/; do
    # Upewnienie się, że iterujemy po fizycznych folderach
    if [ -d "$d" ]; then
        DIR_NAME=$(basename "$d")
        # Wykluczamy katalog repozytorium z usuwania
        if [ "$DIR_NAME" != "$REPO_DIR_NAME" ]; then
            echo "Usuwanie folderu: $d"
            rm -rf "$d"
        fi
    fi
done

echo "Tworzenie wirtualnego środowiska w $USER_HOME/Mavlink..."
# Wykonanie komend w imieniu użytkownika (np. pi5), a nie jako root!
sudo -u "$USERNAME" bash -c "
    mkdir -p $USER_HOME/Mavlink
    cd $USER_HOME/Mavlink
    python3 -m venv venvMavlink
    chmod u+x ./venvMavlink/bin/activate
    source ./venvMavlink/bin/activate
    pip install -r '$SCRIPT_DIR/requirements.txt'
"

echo "--- ZAKOŃCZONO ---"
echo "Zależności Pythona zostały zainstalowane. Aby zastosować wszystkie zmiany, wykonaj:"
echo "sudo reboot"
