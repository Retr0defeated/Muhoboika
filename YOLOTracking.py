import cv2
import numpy as np
import time
import pigpio
import torch
from ultralytics import YOLO

# Настройки сервоприводов и питания
SERVO_X_GPIO = 23  # Серво X (лево-право)
SERVO_Y_GPIO = 24  # Серво Y (вверх-вниз)
POWER_GPIO = 17    # Управление питанием (включается при обнаружении объекта)

# Начальные углы сервоприводов
angle_x = 90
angle_y = 90

# Подключение к pigpio
pi = pigpio.pi()
if not pi.connected:
    exit("Error: pigpio is not running. Start it with: sudo systemctl start pigpiod")

# Настройка GPIO питания
pi.set_mode(POWER_GPIO, pigpio.OUTPUT)
pi.write(POWER_GPIO, 0)  # Изначально выключено

# Загрузка модели YOLO
model = YOLO("yolov8n.pt")  # Используем предобученную YOLOv8 nano


def set_servo_angle(pin, angle):
    """Устанавливает угол сервопривода (500-2500 мкс)"""
    pulse_width = 500 + (angle * 2000 // 180)
    pi.set_servo_pulsewidth(pin, pulse_width)

# Подключение к USB-камере
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Ширина кадра
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # Высота кадра

if not cap.isOpened():
    exit("Error: Cannot open USB camera")

try:
    while True:
        # Захват кадра
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Обнаружение объектов с помощью YOLO
        results = model(frame)
        detections = results[0].boxes.data.cpu().numpy()  # Получаем массив координат

        if len(detections) > 0:
            # Выбираем самый большой объект
            detections = sorted(detections, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]), reverse=True)
            x1, y1, x2, y2, conf, cls = detections[0]  # Координаты, уверенность, класс

            # Найти центр объекта
            obj_x = int((x1 + x2) / 2)
            obj_y = int((y1 + y2) / 2)

            # Вычисление отклонения от центра кадра
            frame_center_x = 640 // 2
            frame_center_y = 480 // 2
            offset_x = obj_x - frame_center_x
            offset_y = obj_y - frame_center_y

            # Коррекция углов сервоприводов
            sensitivity = 0.1
            angle_x -= int(offset_x * sensitivity)
            angle_y -= int(offset_y * sensitivity)

            # Ограничение диапазона 0-180°
            angle_x = max(0, min(180, angle_x))
            angle_y = max(0, min(180, angle_y))

            print(offset_x, offset_y, angle_x, angle_y)

            # Установка новых углов сервоприводов
            set_servo_angle(SERVO_X_GPIO, angle_x)
            set_servo_angle(SERVO_Y_GPIO, angle_y)

            # Включение питания при обнаружении объекта
            pi.write(POWER_GPIO, 1)
        else:
            # Отключение питания при отсутствии объекта
            pi.write(POWER_GPIO, 0)

        # Отображение изображения (опционально)
        cv2.imshow("USB Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(3)  # Задержка перед следующим кадром

except KeyboardInterrupt:
    print("\nПрерывание скрипта пользователем (Ctrl+C). Завершение...")

except Exception as e:
    print(f"\nПроизошла ошибка: {e}")

finally:
    print("Остановка сервоприводов и освобождение ресурсов...")
    pi.set_servo_pulsewidth(SERVO_X_GPIO, 0)
    pi.set_servo_pulsewidth(SERVO_Y_GPIO, 0)
    pi.write(POWER_GPIO, 0)  # Выключение питания
    pi.stop()
    cap.release()
    cv2.destroyAllWindows()
    print("Очистка завершена. Выход.")
