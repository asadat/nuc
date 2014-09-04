cd tables
echo "set terminal pdf size 15cm, 10cm" > gp
echo "set output 'graph.pdf'" >> gp
echo "set multiplot layout 2,2" >>gp
for P in `seq 1 1 4`
do
echo "set title font \"Time-Roman, 5\"">>gp
echo "set key left">>gp 
echo "set yrange [0:9000]">>gp
echo "set xrange [0:100]">>gp
echo "set title \"$P intresting patches\"" >>gp
echo "set key font \"Time-Roman, 4\"">>gp
echo "set key at -10, 9000">>gp
echo "set xtics font \"Time-Roman, 4\"">>gp
echo "set ytics font \"Time-Roman, 4\"">>gp
echo "set xlabel font \"Time-Roman, 4\"">>gp
echo "set ylabel font \"Time-Roman, 4\"">>gp
echo "set xlabel '% of interesting regions'" >>gp
echo "set ylabel 'path length (m)'">>gp
echo "set xtics (\"10%%\" 10, \"20%%\" 20, \"30%%\" 30, \"40%%\" 40, \"50%%\" 50, \"60%%\" 60, \"70%%\" 70, \"80%%\" 80, \"90%%\" 90, \"100%%\" 100)">>gp
echo "set ytics ( \"1Km\" 1000, \"2km\" 2000, \"3km\" 3000, \"4km\" 4000, \"5km\" 5000, \"6km\" 6000, \"7km\" 7000, \"8km\"  8000, \"9km\"  9000, \"10km\" 10000)">>gp
echo "f(x)=4180">>gp
#echo "set xlabel \"% of the environment that is interesting\" font \",8\"">>gp
#echo "set ylabel \"Length of the coverage path (m)\" font \",8\"">>gp
echo "set pointsize 1">>gp
echo "plot 'df' using 2:(\$1==$P?\$3:1/0) with lines notitle, \\">>gp
echo       "'sc' using 2:(\$1==$P?\$3:1/0) with lines notitle, \\">>gp
echo       "'hi' using 2:(\$1==$P?\$3:1/0) with lines notitle, \\">>gp
echo       "'df' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars title 'Depth-First', \\">>gp
echo       "'sc' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars title 'Shortcut', \\">>gp
echo       "'hi' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 8 title 'Hilbert', \\">>gp
echo 	   "f(x) with lines title 'Lawnmower'"	     >>gp
done
echo "unset multiplot">>gp

gnuplot gp
evince graph.pdf &
cd ..      
       
       
       
