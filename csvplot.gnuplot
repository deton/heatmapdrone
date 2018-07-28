set datafile separator comma
set xlabel "x [m]"
set ylabel "y [m]"
set zlabel "温度[℃]" offset 5,3.3
splot "sampleresult.csv" with linespoints
