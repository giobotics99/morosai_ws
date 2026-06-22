import cv2
import apriltag
from pupil_apriltags import Detector
import os

# Cartella con i frame salvati
folder = "/home/orion/morosai_mini_ws/frames_bgr"
images = sorted([f for f in os.listdir(folder) if f.endswith(".png") or f.endswith(".jpg")])

detector = Detector(families="tagStandard52h13")

for img_file in images:
    path = os.path.join(folder, img_file)
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)  # Apriltag lavora su grayscale

    # Rileva tag
    results = detector.detect(img)
    print(f"{img_file}: {len(results)} tag trovati")

    # Disegna bounding box e ID dei tag
    for r in results:
        (ptA, ptB, ptC, ptD) = r.corners
        ptA = tuple(map(int, ptA))
        ptB = tuple(map(int, ptB))
        ptC = tuple(map(int, ptC))
        ptD = tuple(map(int, ptD))
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        cv2.putText(img, str(r.tag_id), ptA, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

    # Mostra il frame
    cv2.imshow("Apriltag Detection", img)
    key = cv2.waitKey(0)  # premi un tasto per passare al frame successivo

cv2.destroyAllWindows()