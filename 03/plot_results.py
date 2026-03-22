import csv
import sys
from PIL import Image, ImageDraw

def get_color(temp, t_min, t_max):
    # Normalized temperature [0, 1]
    norm = (temp - t_min) / (t_max - t_min) if t_max > t_min else 0.5
    # BGR-ish mapping: 0 -> Blue (255,0,0), 1 -> Red (0,0,255)
    r = int(255 * norm)
    b = int(255 * (1.0 - norm))
    g = 0
    return (r, g, b)

def plot(csv_file, output_image):
    width, height = 800, 600
    img = Image.new('RGB', (width, height), 'white')
    draw = ImageDraw.Draw(img)

    # Read CSV
    data = []
    t_vals = []
    v_conv_vals = []
    v_zener_vals = []

    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            try:
                t = float(row[0])
                v_conv = float(row[3])
                v_zener = float(row[4])
                data.append((t, v_conv, v_zener))
                t_vals.append(t)
                v_conv_vals.append(v_conv)
                v_zener_vals.append(v_zener)
            except (ValueError, IndexError):
                continue

    if not data:
        print("No data to plot")
        return

    t_min, t_max = min(t_vals), max(t_vals)
    vx_min, vx_max = 0, max(v_conv_vals) * 1.1
    vy_min, vy_max = 0, max(v_zener_vals) * 1.1

    # Draw grid
    for v in range(0, int(vx_max) + 1, 5):
        x = int(v / vx_max * width)
        draw.line([(x, 0), (x, height)], fill='lightgray')
    for v in range(0, int(vy_max) + 1, 5):
        y = height - int(v / vy_max * height)
        draw.line([(0, y), (width, y)], fill='lightgray')

    # Group by temperature and plot
    datasets = {}
    for t, vx, vy in data:
        if t not in datasets:
            datasets[t] = []
        datasets[t].append((vx, vy))

    for t in sorted(datasets.keys()):
        points = datasets[t]
        color = get_color(t, t_min, t_max)

        pixel_points = []
        for vx, vy in points:
            px = int(vx / vx_max * width)
            py = height - int(vy / vy_max * height)
            pixel_points.append((px, py))

        if len(pixel_points) > 1:
            draw.line(pixel_points, fill=color, width=2)
        else:
            draw.point(pixel_points, fill=color)

    # Labels (simple)
    draw.text((10, 10), "Zener Diode Characterization", fill='black')
    draw.text((10, height - 20), f"X: Supply (0-{vx_max:.1f}V)   Y: Zener (0-{vy_max:.1f}V)", fill='black')
    draw.text((width - 150, 10), f"Temp: {t_min:.0f}C (Blue) -> {t_max:.0f}C (Red)", fill='black')

    img.save(output_image)
    print(f"Plot saved to {output_image}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 plot_results.py <input.csv> <output.png>")
    else:
        plot(sys.argv[1], sys.argv[2])
