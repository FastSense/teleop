# Teleop node

Модуль для телеуправления разлиными роботами.

ROS2

# Ноды

## keyborad_listener

Нода для мониторинга клавиатуры. Импользуя бибилиотеку pynput, мониторит нажатые клавиши на клавиатуре, и публикует их по таймеру в топик /keyboard (тип сообщения std_msg/String)

*Пример запуска*

```bash
ros2 launch teleop keyboard_listener.launch.py
```
*Аргументы:*
* update_rate - частота публикации в топик (default: '20')
