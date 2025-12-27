import cv2
import numpy as np

# Función para detectar los colores y calcular el centroide
def detectar_colores(frame):
    # Convertir la imagen de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Ecualización del histograma para mejorar la iluminación y contraste
    frame_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    frame_yuv[:,:,0] = cv2.equalizeHist(frame_yuv[:,:,0])  # Ecualización solo en el canal de luminancia
    frame = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR)
    
    # Aplicar filtro Gaussiano para reducir el ruido
    frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # Definir los rangos de color en el espacio HSV para rojo, azul y amarillo
    # Rango para el color rojo (dos rangos ya que el rojo se encuentra en los dos extremos del espectro)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Rango para el color azul
    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([130, 255, 255])
    
    # Rango para el color amarillo
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])

    # Crear las máscaras para los tres colores
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Combinar las máscaras
    mask_combined = cv2.bitwise_or(mask_red, mask_blue)
    mask_combined = cv2.bitwise_or(mask_combined, mask_yellow)
    
    # Aplicar la máscara sobre la imagen original
    result = cv2.bitwise_and(frame, frame, mask=mask_combined)

    # Encontrar los contornos
    contours, _ = cv2.findContours(mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Calcular el centroide de los objetos detectados
    centroids = []
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filtrar contornos pequeños
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                centroids.append((cX, cY))
                # Dibujar el centroide en la imagen
                cv2.circle(result, (cX, cY), 7, (0, 0, 0), -1)

    return result, mask_combined, centroids

def main():
    # Abrir la cámara web (0 es la cámara predeterminada)
    cap = cv2.VideoCapture(0)

    while True:
        # Leer el siguiente fotograma de la cámara
        ret, frame = cap.read()
        if not ret:
            print("No se puede leer el frame.")
            break
        
        # Llamar a la función para detectar los colores
        result, mask, centroids = detectar_colores(frame)
        
        # Mostrar el resultado en una ventana
        cv2.imshow('Colores Detectados', result)  # Imagen con los colores detectados
        cv2.imshow('Máscara', mask)               # Solo la máscara de los colores detectados
        
        # Imprimir el centroide en la consola (opcional)
        if centroids:
            print(f"Centroides detectados: {centroids}")
        
        # Salir del bucle si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar la cámara y cerrar las ventanas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
