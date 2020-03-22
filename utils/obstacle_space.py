# Import necessary standard libraries
import cv2
import numpy as np
# Import necessary constants
from utils.constants import scaling_factor, width, height


def get_slopes(points):
    """
    Get slope of each edge of the polygon
    Polygon can have either have 4 or 6 edges
    :param points: coordinates of the polygon
    :return: a list of slopes of the edges of the polygon
    """
    # Get no. of points
    points_length = len(points)
    i = 0
    # Define an empty list to store slopes of all edges
    slopes = []
    while i < points_length:
        # Get indices of the two points of the edge
        if i != points_length - 1:
            j = i + 1
        else:
            j = 0
        # Calculate slope and append it to the list
        slopes.append((points[j][1] - points[i][1]) / (points[j][0] - points[i][0]))
        i += 1

    return slopes


class Map:
    def __init__(self, radius, clearance):
        """
        Initialize map class with radius of the robot and clearance
        :param radius: radius of the robot in integers
        :param clearance: minimum distance between the robot and the obstacle in integers
        """
        # Various class parameters
        deg_30 = np.pi / 6
        deg_60 = np.pi / 3
        self.height = scaling_factor * height
        self.width = scaling_factor * width
        self.thresh = scaling_factor * (radius + clearance)
        # Coordinates of the convex polygon
        self.coord_polygon = scaling_factor * np.array([(20, height - 120),
                                                        (25, height - 185),
                                                        (75, height - 185),
                                                        (100, height - 150),
                                                        (75, height - 120),
                                                        (50, height - 150)],
                                                       dtype=np.int32)
        # Coordinates of the rectangle
        self.coord_rectangle = scaling_factor * np.array([(95 - 75 * np.cos(deg_30), height - 75 * np.sin(deg_30) -
                                                           30),
                                                          (95 - 75 * np.cos(deg_30) + 10 * np.cos(deg_60), height
                                                           - 75 * np.sin(deg_30) - 10 * np.sin(deg_60) - 30),
                                                          (95 + 10 * np.cos(deg_60), height - 10 * np.sin(deg_60) -
                                                           30),
                                                          (95, height - 30)],
                                                         dtype=np.int32).reshape((-1, 2))
        # Coordinates of the rhombus
        self.coord_rhombus = scaling_factor * np.array([(300 - 75 - (50 / 2), height - (30 / 2) - 10),
                                                        (300 - 75, height - 30 - 10),
                                                        (300 - 75 + (50 / 2), height - (30 / 2) - 10),
                                                        (300 - 75, height - 10)],
                                                       dtype=np.int32).reshape((-1, 2))
        # Get slopes of all the edges of the convex polygon, rectangle, and rhombus
        self.slopes_poly = get_slopes(self.coord_polygon)
        self.slopes_rect = get_slopes(self.coord_rectangle)
        self.slopes_rhom = get_slopes(self.coord_rhombus)
        # Define parameters of curved obstacles
        self.circle = [scaling_factor * 25, (scaling_factor * 225, scaling_factor * 50)]
        self.ellipse = [(scaling_factor * 40, scaling_factor * 20),
                        (scaling_factor * 150, scaling_factor * (height - 100))]
        # Get image to search for obstacles
        self.check_img = self.erode_image()

    def get_y_values(self, x, slopes, coordinates, edge_count):
        """
        Calculate the y value of the current x from each edge
        :param x: x-coordinate of the current node
        :param slopes:a list of slopes of all edges of the polygon
        :param coordinates: a list of vertices of the polygon
        :param edge_count: no. of edges in the polygon
        :return: a list of all y-values
        """
        # Define an empty list to store all y-values
        dist = []
        for i in range(edge_count):
            # Add or subtract threshold based on the index of edge
            if i < (edge_count / 2):
                dist.append(slopes[i] * (x - coordinates[i][0]) + coordinates[i][1] - self.thresh)
            else:
                dist.append(slopes[i] * (x - coordinates[i][0]) + coordinates[i][1] + self.thresh)
        # Return the list of y-values
        return dist

    def check_circle(self, x, y):
        """
        Method to check whether point lies within the circle
        :param x: x-coordinate of the current node
        :param y: y-coordinate of the current node
        :return: true if point lies within the circle
        """
        # Define center of the circle
        a = self.circle[1][0]
        b = self.circle[1][1]
        # Define radius of the circle
        r = self.circle[0] + self.thresh
        # Check using the equation of the circle
        if (x - a) ** 2 + (y - b) ** 2 <= r ** 2:
            return True

        return False

    def check_ellipse(self, x, y):
        """
        Method to check whether point lies within the ellipse
        :param x: x-coordinate of the current node
        :param y: y-coordinate of the current node
        :return: true if point lies within the ellipse
        """
        # Define axes length of the ellipse
        a = self.ellipse[0][0] + self.thresh
        b = self.ellipse[0][1] + self.thresh
        # Define center of the ellipse
        center_a = self.ellipse[1][0]
        center_b = self.ellipse[1][1]
        # Check using the equation of the ellipse
        if ((x - center_a) / a) ** 2 + ((y - center_b) / b) ** 2 <= 1:
            return True

        return False

    def check_polygons(self, x, y):
        """
        Method to check whether point lies within the convex polygon or the quadrilaterals
        :param x: x-coordinate of the current node
        :param y: y-coordinate of the current node
        :return: true if point lies within the convex polygon or the quadrilaterals
        """
        # Get y values for each edge of the convex polygon
        y_poly = self.get_y_values(x, self.slopes_poly, self.coord_polygon, 6)
        last_poly_slope = ((self.coord_polygon[2][1] - self.coord_polygon[5][1]) /
                           (self.coord_polygon[2][0] - self.coord_polygon[5][0]))
        y_poly.append(last_poly_slope * (x - self.coord_polygon[5][0]) + self.coord_polygon[5][1] + self.thresh)
        # Get y values for each edge of the rectangle
        y_rect = self.get_y_values(x, self.slopes_rect, self.coord_rectangle, 4)
        # Get y values for each edge of the rhombus
        y_rhom = self.get_y_values(x, self.slopes_rhom, self.coord_rhombus, 4)
        # Return true if point lies within the convex polygon
        if y_poly[0] <= y <= y_poly[6] and y_poly[1] <= y <= y_poly[5]:
            return True
        elif y_poly[2] <= y <= y_poly[4] and y_poly[6] <= y <= y_poly[3]:
            return True
        # Return true if point lies within the tilted rectangle
        elif y_rect[0] <= y <= y_rect[2] and y_rect[1] <= y <= y_rect[3]:
            return True
        # Return true if point lies within the rhombus
        elif y_rhom[0] <= y <= y_rhom[3] and y_rhom[1] <= y <= y_rhom[2]:
            return True

        return False

    def check_node_validity(self, x, y):
        """
        Method to check whether point lies within any obstacle
        :param x: x-coordinate of the current node
        :param y: y-coordinate of the current node
        :return: false if point lies within any obstacle
        """
        # Check whether the current node lies within the map
        if x >= self.width or y >= self.height:
            return False
        # Check whether the current node lies within any obstacle
        elif self.check_img[y, x].all() == 0:
            return False
        # elif self.check_polygons(x, y) or self.check_circle(x, y) or self.check_ellipse(x, y):
        #     return False

        return True

    def erode_image(self):
        """
        Get eroded image to check for obstacles considering the robot radius and clearance
        :return: image with obstacle space expanded to distance threshold between robot and obstacle
        """
        # Get map with obstacles
        eroded_img = self.get_map()
        # Erode map image for rigid robot
        if self.thresh:
            kernel_size = (self.thresh * 2) + 1
            erode_kernel = np.ones((kernel_size, kernel_size), np.uint8)
            eroded_img = cv2.erode(eroded_img, erode_kernel, iterations=1)

        return eroded_img

    def get_map(self):
        """
        Draw map using various opencv methods
        :return: image with all obstacles
        """
        # Create empty image and fill it with white background
        img = np.zeros((scaling_factor * height, scaling_factor * width, 3), dtype=np.uint8)
        img.fill(255)
        # Define color as a tuple in BGR format for obstacles
        color = (0, 0, 0)
        # Draw obstacles in black color
        cv2.fillPoly(img, [self.coord_polygon], color)
        cv2.fillConvexPoly(img, self.coord_rectangle, color)
        cv2.fillConvexPoly(img, self.coord_rhombus, color)
        cv2.circle(img, self.circle[1], self.circle[0], color, -1)
        cv2.ellipse(img, self.ellipse[1], (self.ellipse[0][0], self.ellipse[0][1]), 0, 0, 360, color, -1)

        return img
