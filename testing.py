import cv2
from RRT import rapidlyExploringRandomTree

start_point = [448, 448]
end_point = [180, 34]


def main():

    image = cv2.imread(
        "D:\\Coding\\Python\\irgroup1\\final_map.png", cv2.IMREAD_GRAYSCALE)
    print("Started Processing...")
    path = rapidlyExploringRandomTree(image, start_point, end_point)
    print(path)
    #solution_map = path_planning(start_point, end_point, image)
    #cv2.imshow("Drawn Map", draw_path(image, solution_map))
    # cv2.waitKey(0)


main()
