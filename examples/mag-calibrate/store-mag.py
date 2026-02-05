import sys
import re
import csv

# Regex for magnetometer lines
mag_re = re.compile(
    r"Magnetometer:\s*\[\s*([-+\d\.eE]+)\s*,\s*([-+\d\.eE]+)\s*,\s*([-+\d\.eE]+)\s*\]"
)

output_file = f"mag_out.csv"

with open(output_file, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["x", "y", "z"])
    print("Storing magnetic data\n")

    count = 0
    for line in sys.stdin:
        match = mag_re.search(line)
        if match:
            x, y, z = map(float, match.groups())
            writer.writerow([x, y, z])
            count += 1

print(f"Saved {count} magnetometer samples to {output_file}")