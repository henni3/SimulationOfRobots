set xrange [-2:2]
set yrange [-2:2]
plot "trajectory.dat" with vectors title "robot trajectory", "world.dat" with lines title "world trajectory"
