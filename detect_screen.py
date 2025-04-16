import torch
import cv2
import numpy as np
import mss
import ctypes
import time
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

model = torch.hub.load('.', 'yolov5s', source='local')
model.conf = 0.4
model.classes = [0]  # uniquement "person"

WIDTH, HEIGHT = 1280, 720
MIN_BOX_HEIGHT = 120
SNAP_DISTANCE = 120  # distance max entre souris et tête pour activer le snap
BREAK_DISTANCE = 200  # distance max autorisée avant d'arrêter le snap

last_target = None

def move_mouse(x, y):
    ctypes.windll.user32.SetCursorPos(x, y)
    print(f"🎯 Souris déplacée à ({x}, {y})")

def capture_screen():
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

            cx = int((x1 + x2) / 2)
            cy = int(y1 + (box_height * 0.15))  # vise un peu sous la tête
            target_x = cx + offset_x
            target_y = cy + offset_y

            cursor = ctypes.wintypes.POINT()
            ctypes.windll.user32.GetCursorPos(ctypes.byref(cursor))
            mouse_x, mouse_y = cursor.x, cursor.y
            dx = target_x - mouse_x
            dy = target_y - mouse_y
            dist = int(np.hypot(dx, dy))

            print(f"🧠 Joueur détecté ({target_x},{target_y}) | Distance souris = {dist}px")

            if dist < SNAP_DISTANCE:
                move_mouse(target_x, target_y)
                last_target = (target_x, target_y)
            elif dist > BREAK_DISTANCE:
                print("🛑 Mouvement rapide ou joueur perdu — snap désactivé")
                last_target = None

            # Affichage
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            break  # 1 seul joueur géré à la fois

    else:
        last_target = None  # personne détectée

    cv2.imshow("🎯 Aim Assist Debug", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
