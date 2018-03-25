set terminal postscript color eps enhanced "Times" #solid #"NimbusSanL-Regu"

set key left top
set style data line

# make figures easier to read
set style line 1 lw 2
set style line 2 lw 2
set style line 3 lw 2
set style line 4 lw 2
set style line 5 lw 2
set style line 6 lw 2

set size 0.65,0.40
set xlabel "Time step"

set key invert left top invert
set output "timing.eps"
set ylabel "Cumulative time (s)"
plot "<awk '{print i,sum; i++; sum+=$1}' tgn.txt" title "iSAM" ls 1,\
     "<awk '{print i,sum; i++; sum+=$1}' tsrs.txt" title "Gauss-Newton" ls 3
