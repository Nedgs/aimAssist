import torch
import cv2
import numpy as np
import mss
import ctypes
import ctypes.wintypes
import time
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

# Chargement YOLOv5
model = torch.hub.load('.', 'yolov5s', source='local')
model.conf = 0.4
model.classes = [0]  # uniquement "person"

# Paramètres de la zone de capture
WIDTH, HEIGHT = 1280, 720
MIN_BOX_HEIGHT = 120
SNAP_DISTANCE = 120
BREAK_DISTANCE = 200
AIM_Y_OFFSET = 0.35  # 0.35 = haut du torse, 0.15 = tête

def move_mouse(x, y):
    screen_width = ctypes.windll.user32.GetSystemMetrics(0)
    screen_height = ctypes.windll.user32.GetSystemMetrics(1)

    abs_x = int(x * 65535 / screen_width)
    abs_y = int(y * 65535 / screen_height)

    class MouseInput(ctypes.Structure):
        _fields_ = [
            ("dx", ctypes.c_long),
            ("dy", ctypes.c_long),
            ("mouseData", ctypes.c_ulong),
            ("dwFlags", ctypes.c_ulong),
            ("time", ctypes.c_ulong),
            ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong)),
        ]

    class Input(ctypes.Structure):
        _fields_ = [("type", ctypes.c_ulong), ("mi", MouseInput)]

    mi = MouseInput(abs_x, abs_y, 0, 0x0001 | 0x8000, 0, None)  # MOVE + ABSOLUTE
    inp = Input(0, mi)

    ctypes.windll.user32.SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))
    print(f"🎯 [SendInput] Souris déplacée à ({x}, {y}) → abs=({abs_x}, {abs_y})")


def capture_screen():
    """Capture une zone centrale de l’écran"""
    with mss.mss() as sct:
        monitor = sct.monitors[1]
        center_x = monitor["width"] // 2
        center_y = monitor["height"] // 2
        bbox = {
            "top": center_y - HEIGHT // 2,
            "left": center_x - WIDTH // 2,
            "width": WIDTH,
            "height": HEIGHT
        }
        img = np.array(sct.grab(bbox))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img, bbox["left"], bbox["top"]

print("🟢 Aim Assist intelligent actif — Appuie sur Q pour quitter")

while True:
    frame, offset_x, offset_y = capture_screen()
    results = model(frame)
    detections = results.xyxy[0].cpu().numpy()

    if len(detections) > 0:
        detections = sorted(detections, key=lambda x: (x[2]-x[0])*(x[3]-x[1]), reverse=True)

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            box_height = y2 - y1
            if box_height < MIN_BOX_HEIGHT:
                continue

            # Coordonnées dans la capture
            cx = int((x1 + x2) / 2)
            cy = int(y1 + (box_height * AIM_Y_OFFSET))

            # Coordonnées écran absolues
            target_x = cx + offset_x
            target_y = cy + offset_y

            # Position actuelle de la souris
            cursor = ctypes.wintypes.POINT()
            ctypes.windll.user32.GetCursorPos(ctypes.byref(cursor))
            mouse_x, mouse_y = cursor.x, cursor.y

            dx = target_x - mouse_x
            dy = target_y - mouse_y
            dist = int(np.hypot(dx, dy))

            print(f"🧠 Joueur détecté ({target_x},{target_y}) | Distance souris = {dist}px")

            # Si la souris est proche → snap
            if dist < SNAP_DISTANCE:
                move_mouse(target_x, target_y)
            elif dist > BREAK_DISTANCE:
                print("🛑 Mouvement rapide ou joueur perdu — snap désactivé")

            # Dessin pour affichage
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            break  # un seul joueur traité à la fois

    cv2.imshow("🎯 Aim Assist Debug", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
