# README

## Как пользоваться потенциальными полями для дрона с алгоритмами из drone_ws

1. Запускаем
2. В настройках динамической реконфигурации включаем enable_turn_vectors_to_global
3. Пользуемся

## Описание параметров сервера динамической реконфигурации

- **enable_turn_vectors_to_global** - включает поворот вектора потенциальных поле в глобальную систему координат

- **repulsion_multiplier** - коэффициент отталкивания
- **max_calculation_distance** - максимальная дистанция от точки для расчета отталкивания
- **min_calculation_distance** - минимальная дистанция от точки для расчета отталкивания

- **Параметры с префиксом RVIZ** - включить или выключить визуализацию определенного маркера

- **regulator_mode** - режим регулятора
  1. **Безопасный режим** - либо движение по прямой, либо поворот по рысканию
  2. **Режим голономного робота(дрона)** - движение в двух осях, рыскание не повороачивается
  3. **Одновременный поворот по рысканию и движение**
  4. **Режим для дронов с алгоритмами из drone_ws**
- **max_linear_speed** - максимальная линейная скорость(для режима 1)
- **max_angular_speed** - максимальная угловая скорость(для режима 1)
- **max_angle_to_accept_movement** - минимальная угловая скорость, чтобы начать вращаться(для режима 1)