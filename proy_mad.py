import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Configuración de los pines de los LEDs
GPIO.setmode(GPIO.BCM)
green_led_pin = 17
red_led_pin = 27

# Configurar los pines como salida
GPIO.setup(green_led_pin, GPIO.OUT)
GPIO.setup(red_led_pin, GPIO.OUT)

# Función para detectar frutas no maduras
def detect_unripe_fruit(frame):
    # Convertir la imagen a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Definir los rangos de color para detectar frutas no maduras (verde)
    lower_green = np.array([25, 50, 50])
    upper_green = np.array([85, 255, 255])
    
    # Crear una máscara para detectar los píxeles verdes
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Encontrar los contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    total_area = 0
    
    # Iterar sobre los contornos y sumar las áreas de los más grandes
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 100:  # Filtro para ignorar contornos muy pequeños
            total_area += area
    
    # Obtener el tamaño total de la imagen
    total_pixels = mask.size
    
    # Calcular el porcentaje de área de los contornos verdes con respecto al tamaño total
    green_area_percentage = (total_area / total_pixels) * 100
    
    # Si hay más del 10% de área verde en la imagen, consideramos que la fruta está verde
    if green_area_percentage > 10:
        # La fruta está verde, retorna True
        return True, mask  # Retorna True y la máscara
    else:
        # La fruta no está madura (sin verde), retorna False
        return False, mask  # Retorna False y la máscara

# Estado inicial de la fruta (suponemos que al principio no hay fruta verde)
current_state = None

# Capturar video de la cámara
cap = cv2.VideoCapture(0)

while True:
    # Leer un fotograma del video
    ret, frame = cap.read()
    
    if not ret:
        print("Error: No se pudo leer el fotograma.")
        break
    
    # Detectar frutas no maduras y controlar los LEDs
    is_green, mask = detect_unripe_fruit(frame)
    
    # Mostrar la imagen original y la máscara
    cv2.imshow('Original', frame)  # Muestra la imagen original
    cv2.imshow('Mask', mask)  # Muestra la máscara de los píxeles verdes
    
    # Si la fruta es verde, enciende el LED verde
    if is_green != current_state:
        current_state = is_green  # Actualizar el estado de la fruta
        if is_green:
            # La fruta está verde
            GPIO.output(green_led_pin, GPIO.HIGH)
            GPIO.output(red_led_pin, GPIO.LOW)
            print("Fruta verde detectada. LED verde encendido.")
            time.sleep(5)  # Mantener el LED verde por 5 segundos
            GPIO.output(green_led_pin, GPIO.LOW)  # Apagar el LED verde
        else:
            # La fruta está madura (no verde)
            GPIO.output(green_led_pin, GPIO.LOW)
            GPIO.output(red_led_pin, GPIO.HIGH)
            print("Fruta madura detectada. LED rojo encendido.")
            time.sleep(5)  # Mantener el LED rojo por 5 segundos
            GPIO.output(red_led_pin, GPIO.LOW)  # Apagar el LED rojo
    
    # Salir del bucle al presionar 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Limpiar y cerrar
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()  # Limpiar los pines de Raspberry Pi
