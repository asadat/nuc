cd tables
P=1
echo "set terminal pdf size 20cm, 15cm" > gp
echo "set output 'graph.pdf'" >> gp
echo "set key left">>gp 
echo "set yrange [0:500]">>gp
echo "set xrange [0:100]">>gp
echo "set xtics font \"Time-Roman, 7\"">>gp
echo "set ytics font \"Time-Roman, 7\"">>gp
#echo "set xlabel \"% of the environment that is interesting\" font \",8\"">>gp
#echo "set ylabel \"Length of the coverage path (m)\" font \",8\"">>gp
echo "set pointsize 1">>gp
echo "plot 'df' using 2:(\$1==$P?\$3:1/0) with lines title 'Depth-First', \\">>gp
echo       "'sc' using 2:(\$1==$P?\$3:1/0) with lines title 'Shortcut', \\">>gp
echo       "'hi' using 2:(\$1==$P?\$3:1/0) with lines title 'Hilbert', \\">>gp
echo       "'df' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars notitle, \\">>gp
echo       "'sc' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars notitle, \\">>gp
echo       "'hi' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars notitle \\">>gp

gnuplot gp
evince graph.pdf &
cd ..      
       
       
       
