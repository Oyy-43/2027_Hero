# 3508 open-loop identification and PI proposal

## Scope

- Source: 3508.csv, 96,630 samples, 96.629 s, nominal 1 ms sampling.
- Input range: -0.435 to +0.435 software torque units.
- Output: Now_Omega, rad/s under the current driver configuration (Gearbox_Rate=1).
- Assumption: Torque is Sin_Out / the applied torque command, not measured torque.
- Ozone currently samples Sin_Out, but the renamed CSV header alone cannot prove its source.
- Firmware was not modified or flashed. Results below are offline estimates.

## Estimation

The 18 commanded frequencies span 0.5-25 Hz. Each segment excludes its first
max(2 cycles, 0.25 s). Least squares estimates the fundamental with a constant,
linear drift, and harmonics 2-5. The output/input complex ratio is the FRF.
Half-window comparisons check within-segment repeatability, not independent-run
validation. Cross-validation below holds out alternating frequency points.

First-order plus delay fit:

    G1(s) = 667.8 exp(-0.005081 s) / (s + 0.7242)

Equal-relative-weight complex FRF RMS error: 10.30%.

Preferred empirical fit over the measured band:

    G3(s) = 641.74 / (s + 0.63379)
            * 44375 / (s^2 + 172.81 s + 44375)

Equivalent rounded polynomial form:

    G3(s) = 2.84777e7 / (s^3 + 173.444 s^2 + 44485.1 s + 28124.8)

Complex FRF RMS error: 5.11%. Alternating-point holdout errors: 6.13%, 6.62%.
These are FRF errors, not time-domain fit percentages or confidence intervals.
The fitted extra pure delay reaches its lower bound of zero; this does NOT
establish zero physical latency. Pole dynamics can absorb latency in this band.
The high-frequency pole pair has a natural frequency of about 33.5 Hz, outside
the measured band: do not interpret it as a confirmed mechanical resonance.
DC gain and the low pole (about 0.10 Hz) are extrapolated below the measured band.
Do not interpret the numerator inverse as a calibrated rotor inertia.
The driver's torque conversion is 0.3 and has not been physically calibrated.

## PI candidates

Controller: C(s)=Kp+Ki/s. Firmware uses I += Ki * error * dt, dt=0.001 s.
No derivative or feedforward is included in this proposal.
FRF evaluation uses the discrete PI C(z)=Kp+Ki*0.001/(1-z^-1).

| Candidate | Kp | Ki | Kd | Crossover | Phase margin | Max sensitivity in 0.5-25 Hz |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Initial checkout | 0.04 | 0.30 | 0 | 4.27 Hz | 71.9 deg | 1.19 |
| Recommended balance | 0.07 | 0.80 | 0 | 7.58 Hz | 66.0 deg | 1.38 |
| Faster follow-up | 0.08 | 1.00 | 0 | 8.72 Hz | 64.3 deg | 1.45 |

Margins are interpolated from measured frequency points. Unmeasured dynamics
above 25 Hz can introduce another crossover; global stability is not certified.
No claim of global optimality is made: noise, load envelope, thermal limits and
acceptable overshoot are not specified or experimentally validated.

## Nominal model predictions for Kp=0.07, Ki=0.8

Discrete ZOH simulation, 1 ms controller, third-order model, 1 rad/s step:

- 10-90% rise: 20 ms; overshoot: 14.2%; 2% settling: 228 ms.
- Opposing input disturbance of 0.05 software torque units at t=1 s:
  peak speed error 0.568 rad/s; return to +/-0.02 rad/s in 271 ms.
- Adding 2 ms extra actuation delay gives 17 ms rise, 16.3% overshoot,
  224 ms settling and 0.618 rad/s peak disturbance error.
- Simulation command limit +/-0.435, integrator limit +/-0.3; these small-signal
  cases do not saturate. Large speed steps will not follow these predictions.
- No measured noise, Coulomb friction, heating or physical current-loop model
  is included. Closed-loop hardware validation remains necessary.

## Firmware application checklist

1. Ensure PID_Cal() executes inside Class_Motor_DJI_C620::TIM_Calculate_PeriodElapsedCallback().
2. Stop torque sweep injection. Set Target_Torque=0, Feedforward_Torque=0 and use
   Set_Target_Omega(). Ensure only one task writes the motor targets.
3. Keep OMEGA mode. Run PID once per actual 1 ms cycle, using unchanged units
   and the same feedback filtering as the identification experiment.
4. Clear Iout, ITerm, Omega_Out_Torque and other PID history before transition.
5. Initially limit output to +/-0.435 and integral to +/-0.3 software torque units.
   These are conservative test-envelope limits, not rated motor torque limits.
6. Begin with Kp=0.04, Ki=0.3, Kd=Kf=0, small targets and secured mechanics.
   Then evaluate 0.07/0.8 before considering 0.08/1.0.
7. The current PID returns zero output when absolute error <= DeadBand, even
   with nonzero stored integral. With DeadBand=0, exact zero error still takes
   that branch. Inspect this behavior before relying on steady load rejection.
8. Capture target speed, measured speed, applied output and integral while
   performing small steps and repeatable load tests. Stop on oscillation,
   excessive current or temperature. Do not increase the original 5.8 output
   limit without checking current, mechanics and saturation recovery.
9. Repeat sweeps at representative speeds/loads and extend above 25 Hz before
   pursuing substantially higher bandwidth. Verify Torque column provenance.

## Tracking-priority update (2026-11-11)

The user prioritizes faster tracking and accepts some overshoot. Recommend
Kp=0.10, Ki=1.50, Kd=Kf=0 as the next hardware-validation candidate; 0.12/2.0
is a subsequent experimental candidate, not a certified stable setting.

| Kp / Ki | Measured-band crossover / phase margin | Nominal rise | Nominal overshoot | 2% settling | Overshoot with extra 2 ms delay |
| --- | --- | --- | --- | --- | --- |
| 0.07 / 0.8 | 7.58 Hz / 66.0 deg | 20 ms | 14.2% | 228 ms | 16.3% |
| 0.10 / 1.5 | 13.19 Hz / 58.6 deg | 12 ms | 16.7% | 167 ms | 30.9% |
| 0.12 / 2.0 | 15.19 Hz / 54.4 deg | 10 ms | 29.1% | 143 ms | 47.1% |
| 0.12 / 3.0 | 15.44 Hz / 49.4 deg | 10 ms | 38.2% | 108 ms | 58.6% |
| 0.14 / 2.5 | 17.80 Hz / 48.2 deg | 10 ms | 40.2% | 137 ms | 62.1% |

Simulation conditions are the same 1 rad/s step and nominal third-order model
described above. Additional delay is a stress scenario, not measured latency.
At 0.10/1.5, the 0.05 opposing input disturbance produces 0.482 rad/s peak
error and recovers to +/-0.02 rad/s in 193 ms in the nominal simulation.
No closed-loop hardware data has been used to validate any candidate.

Current firmware inspection confirms PID_Cal() is enabled, motor 0 uses
0.07/0.8, and the speed target is an 11 rad/s amplitude, 2.5 s period sine.
Its output/integral limits remain 5.8/1.4, unlike the analysis simulation's
0.435/0.3 limits. The reported small-step simulations do not saturate, but
large-step behavior with the firmware limits has NOT been validated.
Do not increase these limits merely to increase tracking aggressiveness.
Test the proposed gains with small targets first, log target, feedback, output
and integral, and reduce gain on sustained ringing or excessive overshoot.
The analysis script now reproduces the tracking-priority candidates in this
section; the earlier conservative candidates are retained here for context.

## Reproduction

    python Export_data/identify_3508.py

Requires numpy, scipy and matplotlib. Outputs 3508_frf.csv and
3508_identification.png beside the source data.