from pathlib import Path
import re

# Path to your MJCF file
mjcf_path = Path("dropbear_copy.xml")  # <-- replace with your actual MJCF filename

# Read file
lines = mjcf_path.read_text().splitlines()
patched_lines = []

for line in lines:
    if '<joint' in line and 'range=' not in line:
        # Insert range before the closing />
        line = line.rstrip().replace('/>', ' range="-3.14 3.14"/>')
    patched_lines.append(line)

# Save patched file
patched_path = mjcf_path.with_name(mjcf_path.stem + "_patched.xml")
patched_path.write_text('\n'.join(patched_lines))

print(f"✅ Patched MJCF saved as: {patched_path}")
