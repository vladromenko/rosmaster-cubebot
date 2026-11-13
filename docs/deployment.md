# Запуск на ROSMASTER M3 Pro

Исходное окружение — Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 и ROS domain 30. На борту уже установлены нативная прошивка контроллера, слой `~/Explorer` с окружением ROS, драйвером камеры и службами питания, а также пользовательская служба `rosmaster-bridge`. Сохранённый профиль платы использует последовательный порт `/dev/ttyUSB0` и скорость 2 Мбод. Восстанавливать прошивку для обычного обновления CubeBot не требуется.

## Существующий робот

Разместите репозиторий по адресу `~/CubeBot` пользователя `vlad`. Если каталог уже существует, сохраните его целиком перед заменой и верните карты, результаты и свою калибровку. Скрипты ожидают `/home/vlad/Explorer/bin/env.sh` и Python по адресу `~/rosmaster-venv/bin/python`. Путь к Python для панели можно задать через `ROSMASTER_PY`. На роботе камера и ROS 2 доступны из системных Python-пакетов, поэтому при восстановлении окружения используйте `--system-site-packages`.

```bash
python3 -m venv --system-site-packages ~/rosmaster-venv
~/rosmaster-venv/bin/python -m pip install 'numpy==1.26.4' 'fastapi==0.162.0' 'uvicorn[standard]==0.54.0' 'mediapipe==1.2.2' 'pyserial==3.5'
cd ~/CubeBot
```

Проверьте доступность `cv2`, `scipy` и `yaml` в окружении робота. Пакеты ROS и Fast DDS подключает `~/Explorer/bin/env.sh`. Для E3 требуются ROS-пакеты `ros-jazzy-slam-toolbox` и `ros-jazzy-nav2-bringup`.

Для первичной проверки используйте команды ниже. Тестовый запуск панели публикует телеметрию и подавляет команды движения. Он требует ROS и не должен занимать тот же порт, что уже работающая панель.

```bash
./scripts/run.sh --diagnose
./scripts/console.sh --dry-run --port 8091
```

Если служб ещё нет, `./scripts/install_service.sh` устанавливает пользовательские unit-файлы. На исходном роботе для моста также есть drop-in [explorer-startup.conf](deployment/explorer-startup.conf). Службы камеры, питания, конфигурация Fast DDS и собранные ROS-пакеты `arm_msgs`, `orbbec_camera` и `orbbec_camera_msgs` входят в окружение Explorer на роботе. При восстановлении с чистого носителя сначала восстановите это окружение из локальной резервной копии, затем CubeBot и его службы.

```bash
systemctl --user status rosmaster-bridge explorer-camera explorer-power
systemctl --user start rosmaster-console
curl -f http://127.0.0.1:8090/api/state
journalctl --user -u rosmaster-console -n 50
```

Основной адрес исходного робота — `http://192.168.0.177:8090`. Панель рассчитана на локальную сеть и не требует входа. Миссия и живая панель используют одни и те же каналы шасси и руки. Запускайте их по отдельности и освободите управление в Explorer 3.0 до запуска живой панели. Скрипт запуска на время работы останавливает прежние службы `explorer-control` и `explorer-watchdog`, а при выходе возвращает их.

## Зеркало SO-101 Leader

Панель обращается к Explorer 3.0 на Homebrain по адресу `http://192.168.0.168:8030`. Homebrain запускает источник SO-101 Leader, который отправляет положения в WebSocket `/mirror`. Соответствие суставов и калибровка лежат в `config/mirror.json`. Отдельный отправитель находится в `scripts/so101_mirror_sender.py`. Используйте настройки своего комплекта роботов.

## Миссия с кубиком

```bash
./scripts/run.sh --dry-run
./scripts/stop.sh
```

Флаг `--preflight-only` переводит руку в поисковую позу и проверяет камеру, удерживая шасси. Запуск `./scripts/run.sh` без флага выполняет физическую миссию. Рабочие параметры находятся в `cubebot/config.py`, локальные переопределения передаются через `--config`. Перед физическим испытанием отключите зарядку, освободите площадку и держите под рукой выключатель. Для обычной остановки есть STOP в панели и `./scripts/stop.sh`.
