# config_files

Low-level Raspberry Pi configuration fragments.

### TODO 


## Files
- `99-tty-raw.rules`: serial device permissions.
- `cmdline.txt` and `config.txt`: boot-time kernel/camera configuration.
- `requirements.txt`, `apt_packages.txt`, `apt_packages.txt`: package install lists.
- `user_groups.txt`: configure user groups

## Setting network connections pipline

1. How to check current connections?

```{bash}
nmcli connection show
```

2. How to look up connections details in NetworkManager?

```{bash}
sudo ls /etc/NetworkManager/system-connections/
```

3. How to add wifi network with password?

```{bash}
sudo nmcli dev wifi connect "SSID_SIECI" password "HASLO_SIECI"
```

4. How to check priorities?

```{bash}
nmcli -f NAME,UUID,AUTOCONNECT-PRIORITY connection show
```

5. How to set connection priority?

```{bash}
sudo nmcli connection modify "Domowe_WiFi" connection.autoconnect-priority 10
```
