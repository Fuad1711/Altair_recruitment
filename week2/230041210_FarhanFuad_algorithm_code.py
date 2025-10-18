import math

def distance(point1, point2):

    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
# distance between two coordinates
# Algorithm used was Traveling Salesman

def sort(dest):
    if not dest:
        return []
    
    start = dest[0]
    not_visit = dest[1:]
    path = [start]
    current = start
    while not_visit:
        # finding the nearest point using min function with distance as key 
        next_point = min(not_visit, key=lambda p: distance(current, p))
        path.append(next_point)
        not_visit.remove(next_point)
        current = next_point
        # updating current point to next point
    return path[::-1]
    # reversing the path to get from last to first
dest = [(0, 0), (2, 3), (5, 5), (1, 2), (6, 1)]
shortest = sort(dest)
print("Shortest Path:")
print(shortest)
