"""Generation of ArUco marker — Listing 3.4 (complete script)."""
import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker_image = aruco.generateImageMarker(aruco_dict, id=0, sidePixels=400)

cv2.imwrite("marker_0.png", marker_image)
print("Marker generato e salvato come marker_0.png")
