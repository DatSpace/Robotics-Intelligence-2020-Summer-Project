import cv2
from RRT import rapidlyExploringRandomTree

start_point = [448, 448]
end_point = [180, 34]


def main():

    image = cv2.imread(
        "D:\\Coding\\Python\\irgroup1\\final_map.png", cv2.IMREAD_GRAYSCALE)
    print("Started Processing...")
    print(start_point)
    print(end_point)
    print(type(start_point))
    path = rapidlyExploringRandomTree(image, start_point, end_point)
    print(path)


main()
