from testing_suite_ice_rover_Visual import State, Submission
import math
import matplotlib
import matplotlib.pyplot as plt
PI = math.pi
params = {'test_case': 7,
'area_map': ['.............L......................................',
             '.............LL.....................................',
             '..LLLLLLLLLLLLLLL.............L.....................',
             '.............LL...L..........LL.....................',
             '..L..........L....L........LLLL.....................',
             '.LLL..............L.......LLLLL.....................',
             'LLLLL.............L........L........................',
             '..L...............L.......L.........................',
             '..L...............L......L..........................',
             '..L.............LLLLL...L...........................',
             '..L..............LLL...L............................',
             '..L...............L...L.............................',
             '.L@L.................L..............................',
             '..L.................................................'],
'todo': [(2.0, -3.0),(16.5, -2.5),(18.0, -12.5),(32.5, -1.5),(38.5, -3.75),(43.0, -3.75),(35.5, -14.0),(52.0, -14.0)],
'horizon_distance': 3.5,
'max_distance': 3.0,
'max_steering': PI / 2. + 0.01}
if __name__ == "__main__":
    sub = Submission()
    spiral_path = sub.execute_student_plan(params['area_map'], params['todo'], params['max_distance'], params['max_steering'], params['horizon_distance'])
    x_sample, y_sample = zip(*params['todo'])
    x = spiral_path[1:, 0]
    y = spiral_path[1:, 1]
    fig, ax = plt.subplots()
    ax.plot(x, y)
    map = params['area_map']
    # print(map)
    landmarks = []
    for i in range(len(map)):
        for j in range(len(map[0])):
            # print(map[i][j])
            if map[i][j] =='L':
                landmarks.append((j,-i))
    # print(landmarks)
    x_landmarks, y_landmarks = zip(*landmarks)
        # for j in range(len(map[0][0])):
        #     print(map[i][])
    ax.scatter(x_landmarks, y_landmarks, color='green')
    ax.scatter(x_sample, y_sample, color='red', marker='X')
    ax.scatter(x[0], y[0], color='black')
    ax.scatter(x[-1], y[-1], color='blue')
    ax.grid()
    plt.show()