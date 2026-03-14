import matplotlib.pyplot as plt

file_path = "/tmp/timing_log.txt"   # change if needed

time_mpc = []
time_wbc = []
time_total = []

with open(file_path, "r") as f:
    lines = f.readlines()

# skip header
for line in lines[1:]:
    parts = line.strip().split(",")

    time_mpc.append(float(parts[0]))
    time_wbc.append(float(parts[1]))
    time_total.append(float(parts[2]))

cycles = list(range(len(time_mpc)))

# ---- compute averages ----
avg_mpc = sum(time_mpc) / len(time_mpc)
avg_wbc = sum(time_wbc) / len(time_wbc)
avg_total = sum(time_total) / len(time_total)

plt.figure()

plt.plot(cycles, time_mpc, label="MPC (us)")
plt.plot(cycles, time_wbc, label="WBC (us)")
plt.plot(cycles, time_total, label="Total (us)")

# ---- plot averages ----
plt.axhline(avg_mpc, linestyle="--", label=f"MPC avg = {avg_mpc:.1f} us")
plt.axhline(avg_wbc, linestyle="--", label=f"WBC avg = {avg_wbc:.1f} us")
plt.axhline(avg_total, linestyle="--", label=f"Total avg = {avg_total:.1f} us")

plt.xlabel("Iteration")
plt.ylabel("Time (µs)")
plt.title("Controller Timing")
plt.legend()
plt.grid(True)

plt.show()