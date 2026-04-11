## native sensor resolutions

| **Rozdzielczość** | **Proporcje** | **Tryb (Mode)** | **Charakterystyka**                                           |
| ----------------- | ------------- | --------------- | ------------------------------------------------------------- |
| **4608 x 2592**   | 16:9          | Full Resolution | Pełna szczegółowość, wolniejszy odczyt.                       |
| **2304 x 1296**   | 16:9          | 2x2 Binning     | **Twój obecny.** Szybszy, lepszy w słabym świetle.            |
| **1536 x 864**    | 16:9          | 3x3 Binning     | Bardzo szybki, mniejszy detal.                                |
| **2304 x 2592**   | ~1:1          | HDR Mode        | Wysoka rozpiętość tonalna (wymaga specyficznej konfiguracji). |

## 1. Tryby AfRange (Zakres ostrzenia)

| **Wartość** | **Nazwa**  | **Opis działania**                                                                     |
| ----------- | ---------- | -------------------------------------------------------------------------------------- |
| **0**       | **Normal** | Skupia się na obiektach od ok. 1m do nieskończoności. Ignoruje bardzo bliskie obiekty. |
| **1**       | **Macro**  | Skupia się na obiektach bardzo bliskich (kilka-kilkanaście cm).                        |
| **2**       | **Full**   | Przeszukuje cały dostępny zakres (od Macro do Nieskończoności).                        |

# Troubleshooting

1. Sprawdź stan portu (Komenda diagnostyczna)
Wpisz poniższą komendę, aby sprawdzić, czy system w ogóle widzi cokolwiek na magistrali I2C (tam, gdzie "rozmawia" z kamerą):

``` {Bash}
ls /dev/i2c-*
```

Następnie sprawdź, czy sterownik jest w ogóle dostępny w systemie:

``` {Bash}
lsmod | grep imx708
```

2. Sprawdź plik konfiguracyjny

sudo vim /boot/firmware/config.txt

Powinno być:

camera_auto_detect=1

Opcjonalnie można dopisać:
dtoverlay=imx708

lub 
dtoverlay=imx708,cam0 # cam0 oznacza fizyczne miejsce wpięcia kamery, może być cam1

3. Sprzwdzić dostępne kamery

rpicam-hello --list-cameras