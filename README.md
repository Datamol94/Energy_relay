# Energy_relay
Energy Relay — Autonomous Drone Flight with Charging Stations  A project for programming autonomous flight of unmanned aerial vehicles (UAVs) for the engineering competition "Energy Relay" (Energoestafeta), held as part of the "Arkhipelag" educational intensive

Программа автономного полёта для отборочного (заочного) этапа соревнования «Энергоэстафета» на платформе Clover (ROS) с распознаванием цвета зарядных станций в симуляторе Gazebo.

Дрон выполняет взлёт с синей светодиодной индикацией, затем последовательно облетает точки на карте ArUco-меток (зарядные станции), в каждой точке включает индикацию «радуга» на время мониторинга, распознаёт цвет станции по изображению с камеры (`main_camera/image_raw`) и выводит результат в терминал. После определения цвета лента переключается на соответствующий цвет.

`clover_flight.py` — основной скрипт миссии.

1. Взлёт (`navigate_wait`, `frame_id='body'`) со светодиодной индикацией синего цвета.
2. Облёт списка точек `waypoints` в системе координат `aruco_map`.
3. В каждой точке: включение эффекта `rainbow`, пауза для мониторинга, распознавание цвета станции по HSV-маске (`detect_color`), вывод координат и цвета в терминал, установка соответствующего цвета ленты.
4. Посадка (`land`).

Цвет определяется в центральной области кадра (30–70% по ширине и высоте) методом HSV-масок, заданных в `COLOR_RANGES` (красный, зелёный). Порог срабатывания — `MIN_PIXELS = 500` ненулевых пикселей маски.

- ROS + пакет Clover (`clover.srv`, `led/set_effect`)
- Python 3, OpenCV (`cv2`), `cv_bridge`
- Симулятор Gazebo с полем ArUco-меток
