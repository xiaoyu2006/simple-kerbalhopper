import krpc, time

HEIGHT = 200
# Proportional, Integral, Derivative
KP = 0.065
KI = 0.0015
KD = .1
DELTA_T = 0.1


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

    def step(self, current: float, dt: float) -> float:
        error = self.target - current

        p = self.kp * error
        i = self.ki * self.intergral * dt
        d = self.kd * (error - self.last_error) / dt

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
    # We don't aim to control any other vars, only height
    vessel.control.sas_mode = vessel.control.sas_mode.radial
    
    input("Press enter to start.")
    vessel.control.activate_next_stage()

    while True:
        flight = vessel.flight()
        alt = flight.surface_altitude
        throttle = controller.step(
            current=alt,
            dt=DELTA_T
        )
        vessel.control.throttle = throttle
        print(
            "Altitude: {alt} Throttle: {throttle}".format(
                alt=alt,
                throttle=throttle,
            )
        )
        time.sleep(DELTA_T)

if __name__ == "__main__":
    main()
