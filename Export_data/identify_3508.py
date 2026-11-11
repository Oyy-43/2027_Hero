from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import cont2discrete, tf2ss


ROOT = Path(__file__).resolve().parent
TABLE = [(0.5, 6), (0.63, 6), (0.8, 7), (1, 8), (1.25, 8),
         (1.6, 10), (2, 10), (2.5, 12), (3.15, 12), (4, 15),
         (5, 15), (6.3, 20), (8, 20), (10, 25), (12.5, 30),
         (16, 40), (20, 50), (25, 60)]
PI_CANDIDATES = [(0.07, 0.8), (0.10, 1.5), (0.12, 2.0),
                 (0.12, 3.0), (0.14, 2.5)]


def harmonic_fit(time, values, frequency):
    phase = 2 * np.pi * frequency * time
    design = np.column_stack([np.ones_like(time), time - time.mean()] +
                             [term for order in range(1, 6)
                              for term in (np.sin(order * phase), np.cos(order * phase))])
    coefficients = np.linalg.lstsq(design, values, rcond=None)[0]
    fundamental = coefficients[3] - 1j * coefficients[2]
    amplitude = abs(fundamental)
    thd = np.linalg.norm(coefficients[4:]) / max(amplitude, 1e-12)
    residual = np.std(values - design @ coefficients)
    return fundamental, thd, residual, coefficients[0]


def response(parameters, frequency, order):
    gain, pole, delay = parameters[:3]
    laplace = 2j * np.pi * np.asarray(frequency)
    result = gain / (laplace + pole) * np.exp(-delay * laplace)
    if order == 2:
        result /= 1 + parameters[3] * laplace
    if order == 3:
        natural, damping = parameters[3:5]
        result *= natural ** 2 / (laplace ** 2 + 2 * damping * natural * laplace + natural ** 2)
    return result


def main():
    data = np.genfromtxt(ROOT / "3508.csv", delimiter=",", names=True)
    time, torque, omega = data["Time"], data["Torque"], data["Now_Omega"]
    assert np.all(np.diff(time) > 0) and np.all(np.isfinite(omega))
    first_peak = np.argmax(torque[:3000])
    start = time[first_peak] - 0.5
    rows = []
    for frequency, cycles in TABLE:
        end = min(start + cycles / frequency, time[-1])
        keep = (time >= start + max(2 / frequency, 0.25)) & (time < end)
        if keep.sum() < 100:
            break
        local_time = time[keep] - start
        input_phasor, _, _, _ = harmonic_fit(local_time, torque[keep], frequency)
        output_phasor, thd, residual, mean = harmonic_fit(local_time, omega[keep], frequency)
        transfer = output_phasor / input_phasor
        midpoint = (local_time[0] + local_time[-1]) / 2
        halves = []
        for part in (local_time < midpoint, local_time >= midpoint):
            input_half = harmonic_fit(local_time[part], torque[keep][part], frequency)[0]
            output_half = harmonic_fit(local_time[part], omega[keep][part], frequency)[0]
            halves.append(output_half / input_half)
        repeat_error = abs(halves[0] - halves[1]) / abs(transfer)
        rows.append((frequency, abs(transfer), np.angle(transfer, deg=True),
                     thd, residual, mean, repeat_error, transfer.real, transfer.imag))
        start += cycles / frequency
    rows = np.array(rows)
    assert len(rows) == 18
    np.savetxt(ROOT / "3508_frf.csv", rows, delimiter=",", comments="",
               header="frequency_hz,gain,phase_deg,thd_2_to_5,residual_std,mean_speed,half_difference,real,imag")
    frequency = rows[:, 0]
    measured = rows[:, 7] + 1j * rows[:, 8]
    print("frequency gain phase THD residual mean half_difference")
    for row in rows:
        print(" ".join(f"{number:.5g}" for number in row[:7]))
    models = []
    for order in (1, 2, 3):
        initial = [600, 1, 0.005] + ([0.005] if order == 2 else [160, 0.5] if order == 3 else [])
        lower = [1, 0, 0] + ([0] if order == 2 else [50, 0.05] if order == 3 else [])
        upper = [100000, 1000, 0.1] + ([1] if order == 2 else [2000, 3] if order == 3 else [])
        def residual(parameters):
            error = (response(parameters, frequency, order) - measured) / abs(measured)
            return np.concatenate([error.real, error.imag])
        fit = least_squares(residual, initial, bounds=(lower, upper), max_nfev=3000)
        error = np.sqrt(np.mean(abs((response(fit.x, frequency, order) - measured) / measured) ** 2))
        print("MODEL", order, "parameters", fit.x, "relative_complex_rms", error)
        models.append((order, fit.x))
        if order == 3:
            for parity in (0, 1):
                training = np.arange(len(frequency)) % 2 == parity
                def split_residual(parameters):
                    error = (response(parameters, frequency[training], order) - measured[training]) / abs(measured[training])
                    return np.concatenate([error.real, error.imag])
                split_fit = least_squares(split_residual, fit.x, bounds=(lower, upper))
                held_out = (response(split_fit.x, frequency[~training], order) - measured[~training]) / measured[~training]
                print("HELD_OUT", parity, "relative_complex_rms", np.sqrt(np.mean(abs(held_out) ** 2)), "parameters", split_fit.x)
    dense = np.geomspace(0.5, 25, 5000)
    empirical = np.exp(np.interp(np.log(dense), np.log(frequency), np.log(abs(measured))) +
                       1j * np.interp(np.log(dense), np.log(frequency), np.unwrap(np.angle(measured))))
    for proportional, integral in PI_CANDIDATES:
        controller = proportional + integral * 0.001 / (1 - np.exp(-2j * np.pi * dense * 0.001))
        loop = controller * empirical
        crossings = np.flatnonzero(np.diff(np.sign(abs(loop) - 1)))
        print("PI", proportional, integral, "crossovers_hz", dense[crossings],
              "PM_deg", 180 + np.angle(loop[crossings], deg=True),
              "Ms_measured_band", np.max(abs(1 / (1 + loop))),
              "S_at_1Hz", abs(1 / (1 + loop[np.argmin(abs(dense - 1))])) )
    gain, pole, _, natural, damping = models[-1][1]
    numerator = [gain * natural ** 2]
    denominator = np.polymul([1, pole], [1, 2 * damping * natural, natural ** 2])
    print("THIRD_ORDER_COEFFICIENTS", numerator, denominator)
    state_a, state_b, state_c, state_d, _ = cont2discrete(tf2ss(numerator, denominator), 0.001)
    for proportional, integral in PI_CANDIDATES:
        for delay_samples in (0, 2):
            simulation_time = np.arange(2001) * 0.001
            speed = np.zeros_like(simulation_time)
            command = np.zeros_like(simulation_time)
            state = np.zeros(3)
            accumulator = 0.0
            for index in range(len(simulation_time) - 1):
                speed[index] = (state_c @ state).item()
                error = 1.0 - speed[index]
                accumulator = np.clip(accumulator + integral * error * 0.001, -0.3, 0.3)
                command[index] = np.clip(proportional * error + accumulator, -0.435, 0.435)
                delayed = command[index - delay_samples] if index >= delay_samples else 0.0
                disturbance = -0.05 if simulation_time[index] >= 1.0 else 0.0
                state = state_a @ state + state_b[:, 0] * (delayed + disturbance)
            speed[-1] = (state_c @ state).item()
            initial = speed[:1000]
            rise = (np.flatnonzero(initial >= 0.9)[0] - np.flatnonzero(initial >= 0.1)[0]) * 0.001
            outside = np.flatnonzero(abs(initial - 1) > 0.02)
            settling = (outside[-1] + 1) * 0.001 if len(outside) else 0.0
            disturbed = speed[1000:]
            outside_disturbance = np.flatnonzero(abs(disturbed - 1) > 0.02)
            recovery = (outside_disturbance[-1] + 1) * 0.001 if len(outside_disturbance) else 0.0
            print("SIMULATION", proportional, integral, "extra_delay_ms", delay_samples,
                  "rise_10_90_s", rise, "overshoot_percent", (max(initial) - 1) * 100,
                  "settle_2percent_s", settling, "disturbance_peak_rad_s", max(abs(disturbed - 1)),
                  "disturbance_recovery_to_0.02_s", recovery,
                  "peak_command", max(abs(command)))
    figure, axes = plt.subplots(3, 1, figsize=(9, 10))
    axes[0].semilogx(frequency, 20 * np.log10(abs(measured)), "o", label="Measured")
    axes[1].semilogx(frequency, np.unwrap(np.angle(measured)) * 180 / np.pi, "o")
    dense_frequency = np.geomspace(0.5, 25, 500)
    for order, parameters in models:
        predicted = response(parameters, dense_frequency, order)
        axes[0].semilogx(dense_frequency, 20 * np.log10(abs(predicted)), label=f"Order {order}")
        axes[1].semilogx(dense_frequency, np.unwrap(np.angle(predicted)) * 180 / np.pi)
    axes[0].set_ylabel("Magnitude (dB)")
    axes[1].set_ylabel("Phase (deg)")
    axes[2].semilogx(frequency, rows[:, 3] * 100, "o-", label="THD (2nd-5th)")
    axes[2].semilogx(frequency, rows[:, 6] * 100, "o-", label="Half-window difference")
    axes[2].set_ylabel("Percent")
    axes[2].set_xlabel("Frequency (Hz)")
    for axis in axes:
        axis.grid(True, which="both", alpha=0.3)
    axes[0].legend()
    axes[2].legend()
    figure.tight_layout()
    figure.savefig(ROOT / "3508_identification.png", dpi=160)


if __name__ == "__main__":
    main()