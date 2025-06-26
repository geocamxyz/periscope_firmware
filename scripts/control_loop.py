import numpy as np
import matplotlib.pyplot as plt


def generate_scurve_delays(total_steps, min_velocity, max_velocity, max_acceleration):
    """
    Generate true s-curve ramping delay values for stepper motor control.

    The s-curve has three phases for acceleration and three for deceleration:
    1. Jerk-in: Acceleration increases from 0 to max_acceleration
    2. Constant acceleration: Acceleration stays at max_acceleration
    3. Jerk-out: Acceleration decreases from max_acceleration to 0

    Parameters:
    total_steps: Total number of steps
    min_velocity: Minimum velocity (steps/second)
    max_velocity: Maximum velocity (steps/second)
    max_acceleration: Maximum acceleration (steps/second²)

    Returns:
    delays: List of microsecond delay values between steps
    velocities: List of corresponding velocities
    accelerations: List of corresponding accelerations
    """

    # Calculate time to reach max velocity with given acceleration profile
    # For s-curve: time = sqrt(4 * delta_v / max_accel)
    delta_v = max_velocity - min_velocity

    # Time for each phase (jerk phases are equal, accel phase varies)
    t_jerk = np.sqrt(delta_v / max_acceleration)  # Time for jerk-in/out phases

    # Check if we can reach max velocity
    if 2 * t_jerk * max_acceleration > delta_v:
        # Can't reach max acceleration - reduce it
        t_jerk = np.sqrt(delta_v / (2 * max_acceleration))
        t_accel = 0
        actual_max_accel = delta_v / (2 * t_jerk ** 2)
    else:
        t_accel = (delta_v - max_acceleration * t_jerk ** 2) / max_acceleration
        actual_max_accel = max_acceleration

    # Total acceleration time
    t_total_accel = 2 * t_jerk + t_accel

    # Calculate step timing
    dt = 0.001  # Time resolution (seconds)
    time_points = np.arange(0, 10, dt)  # Extended time array

    velocities = []
    accelerations = []
    times = []

    current_time = 0
    current_velocity = min_velocity

    # Phase 1: Jerk-in (acceleration increases)
    for t in np.arange(0, t_jerk, dt):
        accel = actual_max_accel * (t / t_jerk) ** 2
        current_velocity += accel * dt
        velocities.append(current_velocity)
        accelerations.append(accel)
        times.append(current_time)
        current_time += dt

    # Phase 2: Constant acceleration
    for t in np.arange(0, t_accel, dt):
        accel = actual_max_accel
        current_velocity += accel * dt
        velocities.append(current_velocity)
        accelerations.append(accel)
        times.append(current_time)
        current_time += dt

    # Phase 3: Jerk-out (acceleration decreases)
    for t in np.arange(0, t_jerk, dt):
        accel = actual_max_accel * (1 - (t / t_jerk) ** 2)
        current_velocity += accel * dt
        velocities.append(current_velocity)
        accelerations.append(accel)
        times.append(current_time)
        current_time += dt

    # Calculate how many steps we need for acceleration
    accel_steps = len(velocities)

    # Determine constant velocity steps
    if accel_steps * 2 >= total_steps:
        # Not enough steps for full profile - scale down
        scale_factor = total_steps / (2 * accel_steps)
        accel_steps = int(accel_steps * scale_factor)
        constant_steps = 0
    else:
        constant_steps = total_steps - 2 * accel_steps

    # Build the complete velocity profile
    complete_velocities = []
    complete_accelerations = []

    # Acceleration phase
    step_indices = np.linspace(0, len(velocities) - 1, accel_steps, dtype=int)
    for i in step_indices:
        complete_velocities.append(velocities[i])
        complete_accelerations.append(accelerations[i])

    # Constant velocity phase
    for _ in range(constant_steps):
        complete_velocities.append(max_velocity)
        complete_accelerations.append(0)

    # Deceleration phase (mirror of acceleration)
    for i in reversed(step_indices):
        complete_velocities.append(velocities[i])
        complete_accelerations.append(-accelerations[i])

    # Convert velocities to delays (microseconds)
    delays = [1_000_000 / v for v in complete_velocities]

    return delays, complete_velocities, complete_accelerations


def plot_scurve_comparison(delays, velocities, accelerations, total_steps):
    """Plot the s-curve profile with proper formatting"""

    steps = list(range(1, len(delays) + 1))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

    # Plot 1: Delay values
    ax1.plot(steps, delays, 'b-', linewidth=2)
    ax1.set_xlabel('Step Number')
    ax1.set_ylabel('Delay (microseconds)')
    ax1.set_title('True S-Curve Ramping: Step Delays')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, total_steps)

    # Plot 2: Velocity profile
    ax2.plot(steps, velocities, 'r-', linewidth=2)
    ax2.set_xlabel('Step Number')
    ax2.set_ylabel('Velocity (steps/second)')
    ax2.set_title('True S-Curve Ramping: Velocity Profile')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, total_steps)

    # Plot 3: Acceleration profile
    ax3.plot(steps, accelerations, 'g-', linewidth=2)
    ax3.set_xlabel('Step Number')
    ax3.set_ylabel('Acceleration (steps/s²)')
    ax3.set_title('True S-Curve Ramping: Acceleration Profile')
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0, total_steps)
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.show()


# Parameters
TOTAL_STEPS = 1600
MIN_VELOCITY = 800  # steps/second
MAX_VELOCITY = 15000  # steps/second
MAX_ACCELERATION = 20000  # steps/second² (you can adjust this!)

# Generate the true s-curve delays
delays, velocities, accelerations = generate_scurve_delays(
    TOTAL_STEPS, MIN_VELOCITY, MAX_VELOCITY, MAX_ACCELERATION
)

# Plot the results
plot_scurve_comparison(delays, velocities, accelerations, TOTAL_STEPS)

# Print statistics
print(f"Total steps: {TOTAL_STEPS}")
print(f"Actual steps generated: {len(delays)}")
print(f"Min velocity: {MIN_VELOCITY} steps/s")
print(f"Max velocity: {MAX_VELOCITY} steps/s")
print(f"Max acceleration: {MAX_ACCELERATION} steps/s²")
print(f"Actual max acceleration: {max(accelerations):.1f} steps/s²")
print(f"Min delay: {min(delays):.1f} μs")
print(f"Max delay: {max(delays):.1f} μs")
print(f"Total time: {sum(delays) / 1_000_000:.3f} seconds")

# Find the phases
accel_phase = []
constant_phase = []
decel_phase = []

for i, accel in enumerate(accelerations):
    if accel > 0.1:
        accel_phase.append(i)
    elif accel < -0.1:
        decel_phase.append(i)
    else:
        constant_phase.append(i)

print(f"\nPhase breakdown:")
print(
    f"Acceleration steps: {len(accel_phase)} (steps {min(accel_phase) + 1 if accel_phase else 0}-{max(accel_phase) + 1 if accel_phase else 0})")
print(f"Constant velocity steps: {len(constant_phase)}")
print(
    f"Deceleration steps: {len(decel_phase)} (steps {min(decel_phase) + 1 if decel_phase else 0}-{max(decel_phase) + 1 if decel_phase else 0})")


# Generate C array output
def generate_c_array(delays, array_name="step_delays"):
    """Generate C array code for microcontroller"""
    c_code = f"// S-curve step delays in microseconds\n"
    c_code += f"// Total steps: {len(delays)}\n"
    c_code += f"// Min delay: {min(delays):.1f} μs, Max delay: {max(delays):.1f} μs\n"
    c_code += f"const uint16_t {array_name}[{len(delays)}] = {{\n"

    # Format delays as integers, 10 per line
    for i in range(0, len(delays), 10):
        line_delays = delays[i:i + 10]
        formatted_delays = [f"{int(delay):4d}" for delay in line_delays]
        c_code += "    " + ", ".join(formatted_delays)
        if i + 10 < len(delays):
            c_code += ","
        c_code += f"  // Steps {i + 1}-{min(i + 10, len(delays))}\n"

    c_code += "};\n"
    return c_code


# Generate and save C array
c_array_code = generate_c_array(delays, "pulse_delays")
print("\n" + "=" * 60)
print("C ARRAY FOR MICROCONTROLLER:")
print("=" * 60)
print(c_array_code)

# Save to header file
with open('../src/scurve_delays.h', 'w') as f:
    f.write("#ifndef SCURVE_DELAYS_H\n")
    f.write("#define SCURVE_DELAYS_H\n\n")
    f.write("#include <stdint.h>\n\n")
    f.write(c_array_code)
    f.write(f"\n#define PULSE_SEQUENCE_LENGTH {len(delays)}\n")
    f.write(f"#define MIN_DELAY_US {int(min(delays))}\n")
    f.write(f"#define MAX_DELAY_US {int(max(delays))}\n\n")
    f.write("#endif // SCURVE_DELAYS_H\n")

print(f"C header file saved as: scurve_delays.h")
print(f"Array size: {len(delays)} elements ({len(delays) * 2} bytes for uint16_t)")

# Optional: CSV for analysis
# with open('scurve_analysis.csv', 'w') as f:
#     f.write("step,delay_us,velocity_steps_per_sec,acceleration_steps_per_sec2\n")
#     for i, (delay, vel, accel) in enumerate(zip(delays, velocities, accelerations)):
#         f.write(f"{i+1},{delay:.2f},{vel:.2f},{accel:.2f}\n")