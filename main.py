import krpc, time

HEIGHT = 200
# Proportional, Integral, Derivative
KP = 0.06
KI = 0.0001
KD = 0.5


class PIDController:
    def __init__(
        self,
        target: float,
        kp: float,
        ki: float,
        kd: float,
    ) -> None:
        self.target = target
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.intergral = 0
        self.last_error = 0

    def step(self, current: float) -> float:
        error = self.target - current

        p = self.kp * error
        i = self.ki * self.intergral
        d = self.kd * (error - self.last_error)

        self.intergral += error
        self.last_error = error

        print(
            "P: {p:.2f} I: {i:.2f} D:{d:.2f}".format(
                p=p, i=i, d=d
            )
        )

        return p + i + d


def main():
    conn = krpc.connect(name="Height PID")
    vessel = conn.space_center.active_vessel
    controller = PIDController(
        target=HEIGHT,
        kp=KP,
        ki=KI,
        kd=KD,
    )
    vessel.control.activate_next_stage()
    # We don't aim to control any other vars, only height
    vessel.control.sas_mode = vessel.control.sas_mode.stability_assist
    while True:
        flight = vessel.flight()
        alt = flight.surface_altitude
        throttle = controller.step(
            current=alt,
        )
        vessel.control.throttle = throttle
        print(
            "Altitude: {alt} Throttle: {throttle}".format(
                alt=alt,
                throttle=throttle,
            )
        )
        time.sleep(0.1)

if __name__ == "__main__":
    main()
