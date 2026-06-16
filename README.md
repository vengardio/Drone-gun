# Drone-gun

Проект прошивки ружья для запуска дрона-перехватчика.

Актуальная рабочая прошивка для платы ружья с поддержкой двух режимов находится в `STM32 DRONE-CYCLOPS`.
Папку `STM32 DRONE` пока не трогаем: она остается отдельной старой/исходной веткой проекта.

## Режимы работы

У прошивки есть два режима.

1. Режим `drone`

   Циклоп не подключался после включения микроконтроллера. Ружье работает напрямую с дроном: читает кнопки и концевик, управляет сервой, отправляет дрону команды по MAVLink через USART2.

2. Режим `cyclops`

   Циклоп хотя бы один раз отправил любой байт по USART1. После этого прошивка навсегда до перезагрузки переключается в режим работы с Циклопом. В этом режиме ружье становится прослойкой: принимает сообщения Циклопа, разбирает их и дальше работает с дроном.

Переключение режима сделано в `app/main.c` через флаг:

```c
static uint8_t cyclops_connected = 0;
```

При любом байте от Циклопа:

```c
void USART1_IRQHandler(void)
{
    if(USART1->SR & USART_SR_RXNE)
    {
        cyclops_connected = 1;
        Cyclops_RxByte((uint8_t)USART1->DR);
    }
}
```

Основной цикл выбирает логику так:

```c
void App_Process(void)
{
    Main_ReadMessages();

    if(cyclops_connected)
    {
        LogicCyclops_Process();
    }
    else
    {
        LogicDrone_Process();
    }
}
```

Флаг обратно в `0` не сбрасывается. Ноль бывает только после старта микроконтроллера.

## Железо и интерфейсы

- MCU: `STM32F103RBT6T`.
- Дрон: `USART2`, `PA2 TX`, `PA3 RX`, `115200`.
- Циклоп: `USART1`, `PA9 TX`, `PA10 RX`, `115200`.
- `PA0` - `LAUNCH`, кнопка запуска.
- `PA7` - `SAFETY`, тумблер разрешения подготовки и запуска.
- `PC4` - `END`, концевик поднятого штока.
- `PA1` - PWM сервы штока, `TIM2_CH2`.
- `PA5` - heartbeat/status LED.
- `PB12`, `PB13` - управление оптопарами.

## Архитектура `STM32 DRONE-CYCLOPS`

Файлы разложены по слоям, чтобы не было кучи исходников в корне.

```text
STM32 DRONE-CYCLOPS/
├── app/
│   ├── main.c
│   └── main.h
├── drivers/
│   ├── gpio.c
│   ├── gpio.h
│   ├── pwm.c
│   ├── pwm.h
│   ├── rcc.c
│   ├── rcc.h
│   ├── timer.c
│   ├── timer.h
│   ├── usart.c
│   └── usart.h
├── protocol/
│   ├── cyclops_rx.c
│   ├── cyclops_rx.h
│   ├── cyclops_tx.c
│   ├── cyclops_tx.h
│   ├── mavlink_rx.c
│   ├── mavlink_rx.h
│   ├── mavlink_tx.c
│   └── mavlink_tx.h
├── logic/
│   ├── logic_cyclops.c
│   ├── logic_cyclops.h
│   ├── logic_drone.c
│   └── logic_drone.h
├── core/
│   ├── message_queue.c
│   └── message_queue.h
├── mavlink/
├── RTE/
├── Objects/
└── Listings/
```

### Назначение папок

- `app/` - входная точка прошивки. Здесь инициализация, основной цикл и обработчики USART-прерываний.
- `drivers/` - работа с периферией STM32: GPIO, PWM, RCC, таймеры, USART.
- `protocol/` - сборка и разбор протоколов. Здесь не должно быть логики поведения ружья.
- `logic/` - логика работы устройства. `logic_drone` отвечает за режим без Циклопа, `logic_cyclops` - за режим с Циклопом.
- `core/` - общие механизмы прошивки. Сейчас здесь очередь сообщений.
- `mavlink/` - внешняя MAVLink-библиотека. Руками без причины не править.

## Поток данных

Прерывания USART принимают байты и отдают их в RX-парсеры:

```text
USART1 IRQ -> cyclops_rx
USART2 IRQ -> mavlink_rx
```

RX-парсеры не принимают решений. Они только собирают валидные сообщения и кладут их в очередь:

```text
cyclops_rx -> message_queue
mavlink_rx -> message_queue
```

Потом `main.c` читает очередь и передает сообщения в нужную логику:

```text
message_queue -> logic_cyclops
message_queue -> logic_drone
```

Отправка наружу идет через TX-модули:

```text
logic_drone   -> mavlink_tx  -> USART2 -> drone
logic_cyclops -> cyclops_tx  -> USART1 -> Cyclops
```

## Очередь сообщений

Очередь нужна, чтобы не делать тяжелую обработку внутри прерываний. В IRQ мы быстро приняли байт, собрали пакет, положили валидное сообщение в очередь и ушли.

Внутренняя структура сообщения:

```c
typedef struct
{
    MessageSource source;
    uint16_t size;
    uint8_t data[MESSAGE_MAX_SIZE];
}Message;
```

`source` не является частью протокола и не меняет пакет. Это служебная метка прошивки:

```c
MESSAGE_SOURCE_DRONE
MESSAGE_SOURCE_CYCLOPS
```

Реальные байты пакета лежат в `data`, размер пакета лежит в `size`.

Чтение очереди в `main.c`:

```c
static void Main_ReadMessages(void)
{
    Message msg;

    while(MessageQueue_Pop(&msg))
    {
        if(msg.source == MESSAGE_SOURCE_CYCLOPS)
            LogicCyclops_ProcessMessage(&msg);

        if(msg.source == MESSAGE_SOURCE_DRONE)
            LogicDrone_ProcessMessage(&msg);
    }
}
```

`Message msg` - локальная временная переменная. В нее копируется сообщение из очереди. Передавать `&msg` в обработчик безопасно, пока обработчик не сохраняет этот указатель на потом.

## Логика без Циклопа

Режим без Циклопа реализуется в `logic/logic_drone.c`.

Текущий сценарий:

1. Ждем поднятый шток: `END = 1`.
2. Удерживаем шток сервой: `Servo_Hold()`.
3. Ждем разрешение от `SAFETY`.
4. При `SAFETY = 1` отправляем дрону `prepare`.
5. Если `SAFETY` выключили - отправляем `cancel`.
6. Если нажали `LAUNCH` - отпускаем шток и отправляем `fly`.

Команды дрону собираются в `protocol/mavlink_tx.c` и отправляются через USART2:

```c
MavlinkTx_SendPrepare();
MavlinkTx_SendCancel();
MavlinkTx_SendFly();
```

Команды MAVLink:

- `31003` - `prepare`.
- `31006` - `cancel`.
- `31009` - `fly`.

## Логика с Циклопом

Режим с Циклопом реализуется в `logic/logic_cyclops.c`.

Сейчас в файле есть точка входа под постоянную логику:

```c
void LogicCyclops_Process(void)
{
}
```

И точка входа для обработки принятых сообщений:

```c
void LogicCyclops_ProcessMessage(Message *msg)
{
    uint8_t cmd;

    if(msg->size < 6)
        return;

    cmd = msg->data[3];

    switch(cmd)
    {
        case 0x81:
            break;

        case 0x82:
            break;

        case 0x83:
            break;

        case 0x84:
            break;

        case 0x85:
            break;

        case 0x86:
            break;

        case 0x87:
            break;

        default:
            break;
    }
}
```

Полноценная логика Циклопа пока не реализована. Ее надо писать отдельно, когда будет понятно, какие команды Циклопа должны что делать с дроном.

## MAVLink

Канал дрона использует MAVLink v2.

Прием идет по USART2. Валидность кадра проверяется через `mavlink_parse_char`, то есть CRC проверяет сама MAVLink-библиотека.

В очередь попадают только валидные MAVLink v2 кадры со стартовым байтом `0xFD`.

## SrvInt / Cyclops

Циклоп работает по протоколу SrvInt.

Формат кадра:

```text
START_BYTE ADR PACKET_ID CMD DATA_COUNT HASH OP_1 ... OP_N DATA_HASH
```

Поля:

- `START_BYTE = 0xB7`.
- `PACKET_ID bit7 = 0`.
- `DATA_COUNT` задает количество байт данных.
- `HASH = XOR(ADR, PACKET_ID, CMD, DATA_COUNT)`.
- Если `DATA_COUNT > 0`, есть `DATA_HASH = XOR(OP_1 ... OP_N)`.
- Если `DATA_COUNT == 0`, `DATA_HASH` отсутствует.

Стоп-байта в протоколе нет. Конец кадра определяется по `DATA_COUNT`.

Базовые команды из протокола:

- `0x00` - `CMD_UNKNOWN`.
- `0x81` - `CMD_PING`.
- `0x82` - `CMD_HW_RESET`.
- `0x83` - `CMD_SW_RESET`.
- `0x84` - `CMD_GET_ERROR`.
- `0x85` - `CMD_ZEROIZE_ERROR`.
- `0x86` - `CMD_SET_PARAM`.
- `0x87` - `CMD_GET_PARAM`.

## Серва штока

Серва управляется из `drivers/pwm.c`.

Функции:

```c
void Servo_Hold(void);
void Servo_Release(void);
```

Значения импульса:

- `Servo_Hold()` - удержать шток, `1740 us`.
- `Servo_Release()` - отпустить шток, `1200 us`.

## План проверки

1. ~~Проверить концевик `END`: когда дрон объективно ожидается, при нажатии концевика надо заткнуть шток через `Servo_Hold()`. Убедиться на железе, что это реально работает.~~

2. Проверить все команды Циклопа по очереди:

   2.1. ~~`PING`: если дрон на месте, возвращаем ping. Если дрона нет, ничего не возвращаем. Проверено.~~

   2.2. ~~`HW_RESET`: отправляем дрону команду hardware reset. Когда получаем первый heartbeat от дрона, возвращаем Циклопу пакет с успешной перезагрузкой.~~

   2.3. ~~`SW_RESET`: отправляем дрону команду software reset и сразу отправляем Циклопу ответ "успешно".~~

   2.4. ~~`GET_ERROR`: при любом раскладе возвращаем 0 ошибок.~~

   2.5. ~~`ZEROIZE_ERROR`: забиваем на обработку ошибок и отправляем Циклопу, что всё хорошо.~~

   2.6. `SET_PARAM`: проверить, что команды установки параметров работают нормально.

   2.7. `GET_PARAM`: уточнить и проверить, чтобы всё было хорошо. Отдельно удостовериться, что интенсивность цели и количество целей берутся из информации от дрона.
