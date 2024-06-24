import RPi.GPIO as GPIO
import time

# Utiliser la numérotation BCM
GPIO.setmode(GPIO.BCM)

# Liste des pins GPIO que nous allons vérifier
gpio_pins = [2, 3, 4, 17, 27, 22, 10, 9, 11, 5, 6, 13, 19, 26, 14, 15, 18, 23, 24, 25, 8, 7, 12, 16, 20, 21]

# Configurer les GPIOs en entrée
for pin in gpio_pins:
    GPIO.setup(pin, GPIO.IN)

# Lire et afficher les états des GPIOs
try:
    while True:
        print("État des GPIOs:")
        for pin in gpio_pins:
            state = GPIO.input(pin)
            print(f"GPIO {pin}: {'HIGH' if state else 'LOW'}")
        # Ajouter une petite pause pour éviter d'inonder la console
        time.sleep(1)

except KeyboardInterrupt:
    # Nettoyer les configurations GPIO avant de sortir
    GPIO.cleanup()
    print("Nettoyage des GPIOs et arrêt du programme")