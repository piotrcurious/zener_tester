# Gnuplot script for Zener Diode Compound Graph
# Usage: gnuplot -p 02/plot_results.gp -e "datafile='results.csv'"

set datafile separator ","
set title "Zener Diode Characteristics at Different Temperatures"
set xlabel "Supply Voltage (V)"
set ylabel "Zener Voltage Drop (V)"
set grid
set key outside

# Use the first column (TargetTemp) for the palette or grouping
# Plotting Zener Voltage (col 5) against Supply Voltage (col 4)
# Each temperature dataset is separated by two blank lines in the CSV for index access
# or we can use the temperature value directly for the color.

plot datafile using 4:5:1 with lines lc variable title "Zener Sweep (grouped by Temp)"
