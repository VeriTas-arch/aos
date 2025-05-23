MODE = "SIM"  # SIM / REAL


class Config:
    def __init__(self):
        self.MODE = MODE

        # speed 对应 `agv_controller.py` 中四个对接调整阶段的速度数值
        self.speed = None
        # tolerance 对应 `agv_controller.py` 中四个对接调整阶段的容差数值
        self.tolerance = None

        if self.MODE == "SIM":
            self.prefix = "modulating_part/"

            self.speed = [0.5, 0.1, 0.1, 0.1]
            self.tolerance = [5.0, 0.2, 0.2, 3.0]

        elif self.MODE == "REAL":
            self.prefix = ""

            self.speed = [3, 1, 1, 3]
            self.tolerance = [5.0, 0.1, 0.1, 3.0]

        else:
            raise ValueError("Invalid mode. Choose 'SIM' or 'REAL'.")

        self.modulatingLink = f"{self.prefix}modulating_Link"
        self.platformLink = f"{self.prefix}platform_Link"
        self.bottomLink = f"{self.prefix}bottom_Link"
        self.cameraLink = f"{self.prefix}xiangji_Link"
