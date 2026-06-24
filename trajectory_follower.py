# from scipy import signal
import numpy as np

from epuck import EPuckController


def main() -> None:
    epuck = EPuckController()
    index = 0

    while epuck.step_simulation():
        # epuck.follow_line()
        epuck.update_odometry()

        rho = epuck.trajectory_following(index)

        if rho < 0.1:
            index += 1
            if index >= len(epuck.waypoints):
                print("Trajectory completed!")
                epuck.is_finished = True

        _, world_point = epuck.lidar2world_coordinate()
        epuck.probabilistic_mapping(world_point)

        px, py = epuck.world2map(world_point[0], world_point[1])

        for x, y in zip(px.tolist(), py.tolist()):
            raw_prob = epuck.map[x, y]
            prob = 1.0 if raw_prob > 0.1 else 0.0

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

    import os

    os.system("cls" if os.name == "nt" else "clear")
