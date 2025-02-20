import cv2
import numpy as np
import imutils
import time
"""import RPi.GPIO as GPIO"""

# ==== Инициализация GPIO для сервоприводов ====
'''GPIO.setmode(GPIO.BOARD)
servo_x_pin = 16
servo_y_pin = 18
GPIO.setup(servo_x_pin, GPIO.OUT)
GPIO.setup(servo_y_pin, GPIO.OUT)

servo_x = GPIO.PWM(servo_x_pin, 50)
servo_y = GPIO.PWM(servo_y_pin, 50)
servo_x.start(7.5)  # Начальная позиция
servo_y.start(7.5)

def set_servo_angle(servo, angle):
    duty = 2.5 + (angle / 18)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.02)'''

# ==== Настройки ====
ACCEPTABLE_X_ERROR = 75
ACCEPTABLE_Y_ERROR = 60
MAX_STEP = 10  # Максимальный шаг за 1 сек
SENSITIVITY = 0.05  # Коэффициент чувствительности для плавного движения
last_move_time = time.time()
angle_x = 90  # Начальный угол сервопривода
angle_y = 90

# ==== Настройка камеры ====
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
time.sleep(1)

# Определение порогов цвета
color_lower = (0, 0, 0)
color_upper = (180, 255, 50)
fgbg = cv2.createBackgroundSubtractorMOG2()
frame_center = None

# ==== Основной цикл ====
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    frame = imutils.resize(frame, 800)
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, color_lower, color_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Фоновая разница
    fg_mask = fgbg.apply(frame)
    moving_mask = cv2.bitwise_and(mask, mask, mask=fg_mask)

    if frame_center is None:
        (h, w) = frame.shape[:2]
        frame_center = (w // 2, h // 2)
    
    contours = cv2.findContours(moving_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)

            if M["m00"] != 0:
                contour_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, contour_center, 5, (0, 0, 255), -1)

                (delta_x, delta_y) = np.subtract(contour_center, frame_center)

                cv2.putText(frame, f"deltaX: {delta_x}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(frame, f"deltaY: {delta_y}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                # === Управление сервоприводами раз в секунду ===
                """current_time = time.time()
                if current_time - last_move_time >= 1:
                    if abs(delta_x) > ACCEPTABLE_X_ERROR:
                        angle_x -= delta_x / 50
                        angle_x = max(0, min(180, angle_x))  # Ограничение угла
                        set_servo_angle(servo_x, angle_x)
                    
                    if abs(delta_y) > ACCEPTABLE_Y_ERROR:
                        angle_y -= delta_y / 50
                        angle_y = max(0, min(180, angle_y))
                        set_servo_angle(servo_y, angle_y)

                    last_move_time = current_time"""  # Обновляем время последнего движения

    cv2.imshow("Frame", imutils.resize(frame, 1100))
    cv2.imshow("Mask", imutils.resize(moving_mask, 1100))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ==== Очистка ресурсов ====
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
