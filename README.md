# Informacje dla Kamila i nie tylko

## Wszelkie funkcje należy założyć że nie działają i wytestować

# PixHawkService

W plikach są różne rzeczy, głównie jakieś testy itp. najciekawsze są funkcje w klasach
W pliku PixHawkService jest kod do komunikacji z matekiem (kiedyś kożystaliśmy z pixhawka to było to samo co matek tylko gorsze)
Najwżniejsze funkcje do działania:
  - **get_current_coordinates**
  - get_mission
  - **set_waypoints** - inne funkcje zmieniające waypointy są niewarte uwagi (nie działały)
  - get_mission_status
  - add_drop_waypoint - tutaj chodzi o inny waypoint który strigeruje serwo do odłączenia ładunku
  - get_attitude
  - set_servo
  - is_armed 


# Mission Service
W pliku MissionService jest klasa tworzona przez klasę PixHawkService, odpowiada za obsługę misji czyli:
kamera wykryła obiekt -> przeczytanie koordynatów samolotu -> naniesienie kordsów z zdjęcia na ziemie -> ustalenie nowej trasy

**Tutaj funkcje są całkiem ok, najciekawszą rzeczą jest wyznaczenie K matrix i chyba dist, są to rzeczy zależne od obiektywu i są wymagane do prawidłowego naniesienia punktu z zdjęcia na powierzchnię ziemii**
