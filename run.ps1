$ErrorActionPreference = "Stop"

python $PSScriptRoot\simple_kinematic_simulator.py; gnuplot -p "$PSScriptRoot\\simple_visualization.gnu"

# python simple_kinematic_simulator.py; gnuplot -p ".\simple_visualization.gnu"