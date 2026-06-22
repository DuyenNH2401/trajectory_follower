# from scipy import signal
from epuck import EPuckController
import os 

def main() -> None:
    epuck = EPuckController()

    index = 0

    while epuck.step_simulation():
        epuck.follow_line()
        epuck.update_odometry()
        
        robot_point, world_point = epuck.lidar2world_coordinate()
        epuck.probabilistic_mapping(world_point)
        
        error = epuck._computing_error(index)
        print(f"Error to waypoint {index}: {error}")

        px, py = epuck.world2map(world_point[0], world_point[1])

        for x, y in zip(px.tolist(), py.tolist()):
            raw_prob = epuck.map[x, y]
            prob = 1.0 if raw_prob > 0.7 else 0.0

            v = int(prob * 255)
            color = int(v * 256**2 + v * 256 + v)
            epuck.display.setColor(color)
            epuck.display.drawPixel(x, y)

        rx, ry = epuck.world2map(epuck.xw, epuck.yw)
        epuck.display.setColor(0xFF0000)
        epuck.display.drawPixel(rx, ry)

    # kernel = np.ones((30, 30))
    # cmap = signal.convolve2d(epuck.map, kernel, mode="same")
    # cspace = cmap > 0.9

    # plt.imshow(cspace)
    # plt.show()


if __name__ == "__main__":
    main()
    os.system('cls' if os.name == 'nt' else 'clear')

