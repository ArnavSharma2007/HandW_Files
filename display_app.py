import time
import board
import RPi.GPIO as GPIO 
import digitalio
from PIL import Image, ImageDraw, ImageFont
from adafruit_rgb_display import st7735

# Firebase Libraries
import firebase_admin
from firebase_admin import credentials, db

# --- 1. DISPLAY HARDWARE CONFIG ---
WIDTH = 128
HEIGHT = 160

CS_PIN = digitalio.DigitalInOut(board.CE0)
DC_PIN = digitalio.DigitalInOut(board.D25)
RST_PIN = digitalio.DigitalInOut(board.D27)
BAUDRATE = 24000000 

# --- 2. FIREBASE CONFIG ---
SERVICE_ACCOUNT_KEY = 'serviceAccountKey.json'
DATABASE_URL = 'https://handwband-default-rtdb.firebaseio.com/'

WEARABLE_PATH = "sensorData"  # Path in Realtime DB

# --- 3. GLOBAL VARIABLES ---
display = None
font = None
image = None
draw = None
flash_state = True

# --- 4. HARDWARE INITIALIZATION ---
def initialize_hardware():
    global display, font, image, draw
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    try:
        spi = board.SPI()
        display = st7735.ST7735R(
            spi, cs=CS_PIN, dc=DC_PIN, rst=RST_PIN,
            baudrate=BAUDRATE, width=WIDTH, height=HEIGHT, rotation=180
        )

        image = Image.new("RGB", (WIDTH, HEIGHT))
        draw = ImageDraw.Draw(image)

        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
        except:
            font = ImageFont.load_default()

        print("Hardware initialized successfully.")
        return True
    except Exception as e:
        print(f"Hardware init error: {e}")
        return False

# --- 5. DRAWING FUNCTION ---
def draw_screen(vitals, state, flash=False):
    if not display or not image or not draw:
        return

    # Clear screen
    draw.rectangle((0, 0, WIDTH, HEIGHT), outline=0, fill=(0, 0, 0))

    if state == "OK":
        fill_color = (160, 160, 160)  # Gray background
        text_color = (0, 0, 0)
        draw.rectangle((0, 0, WIDTH, HEIGHT), outline=0, fill=fill_color)
        draw.text((5, 10), f"TEMP: {vitals.get('BodyTemp', 0):.1f} C", font=font, fill=text_color)
        draw.text((5, 40), f"SpO2: {vitals.get('SpO2', 0)}%", font=font, fill=text_color)
        draw.text((5, 70), f"HR: {vitals.get('HR', 0)} BPM", font=font, fill=text_color)

    elif state == "EMERGENCY":
        fill_color = (255, 0, 0) if flash else (0, 0, 0)  # Flashing red/black
        text_color = (255, 255, 255)
        draw.rectangle((0, 0, WIDTH, HEIGHT), outline=0, fill=fill_color)

        # Center EMERGENCY text using font.getbbox()
        text = "EMERGENCY"
        bbox = font.getbbox(text)
        text_w = bbox[2] - bbox[0]
        text_h = bbox[3] - bbox[1]
        x = (WIDTH - text_w) // 2
        y = (HEIGHT - text_h) // 2
        draw.text((x, y), text, font=font, fill=text_color)

    display.image(image)

# --- 6. FIREBASE INITIALIZATION ---
def initialize_firebase():
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        print("Firebase initialized successfully.")
        return True
    except Exception as e:
        print(f"Firebase init error: {e}")
        return False

# --- 7. MAIN LOOP ---
def main_loop():
    ref = db.reference(WEARABLE_PATH)
    global flash_state

    while True:
        try:
            data = ref.get()
            if not data:
                time.sleep(1)
                continue

            state = data.get('state', 'OK')
            vitals = {
                'HR': data.get('HR', 0),
                'SpO2': data.get('SpO2', 0),
                'BodyTemp': data.get('BodyTemp', 0)
            }

            if state == "EMERGENCY":
                draw_screen(vitals, state, flash=flash_state)
                flash_state = not flash_state
                time.sleep(0.5)  # Flash interval
            else:
                draw_screen(vitals, state)
                time.sleep(1)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error in main loop: {e}")
            time.sleep(2)

if _name_ == '_main_':
    if initialize_hardware() and initialize_firebase():
        main_loop()

    GPIO.cleanup()
