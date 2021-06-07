# Teleop node

Модуль для телеуправления разлиными роботами.

ROS2

## Управление росботом в симуляторе с клавиатуры

Терминал №1 - запускаем сиулятор с росботом
```
ros2 launch rosbot_description rosbot_sim.launch.py
```

Терминал №2 - запускаем телеуправление
```
 ros2 launch teleop rosbot_sim_keyboard_teleop.launch.py
```

# Ноды

## keyborad_listener

Нода для мониторинга клавиатуры. Импользуя бибилиотеку pynput, мониторит нажатые клавиши на клавиатуре, и публикует их по таймеру в топик /keyboard (тип сообщения std_msg/String)

*Пример запуска*

```bash
ros2 launch teleop keyboard_listener.launch.py
```
*Аргументы:*
* update_rate - частота публикации в топик (default: '20')

## rosbot_teleop
Нода для телеуправления росботом. Подписывается на топик /keyboard и интерпретирует команды в управление для росбота. ПУликует управлени в топик /cmd_vel.

*Пример запуска*

```bash
ros2 launch teleop rosbot_teleop.launch.py
```
*Аргументы:*
* update_rate - частота публикации управления в топик (default: '20')

