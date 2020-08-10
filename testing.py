import cv2
from RRT

start_point = [448, 448]
end_point = [34, 180]  # Reversed due to arrays columns/rows 180,34


def main():

    #image = cv2.imread("D:\\Coding\\Python\\irgroup1\\final_map.png", cv2.IMREAD_GRAYSCALE)
    image = cv2.imread(
        "D:\\Coding\\Python\\irgroup1\\original_map.png", cv2.IMREAD_GRAYSCALE)
    # path = find_shortest_path(image, start_point, end_point)
    # for pixel in path:
    #     pixel *= 255
    print("Started Processing...")
    #path = a_star2.astar(image, tuple(start_point), tuple(end_point))
    #path = a_star3.astar(image, tuple(start_point), tuple(end_point))
    #path = a_star4.aStar(image, tuple(start_point), tuple(end_point))
    # print("Showing...")
    #cv2.imshow("Original", image)
    #cv2.imshow("t", path)
    solution_map = path_planning(start_point, end_point, image)
    cv2.imshow("Drawn Map", draw_path(image, solution_map))
    cv2.waitKey(0)


main()
