echo "set terminal pdf size 20cm, 5cm" > gp
echo "set output 'graph.pdf'" >> gp
echo "set lmargin 4">> gp
echo "set multiplot layout 1,4" >>gp
for P in `seq 1 1 4`
do
echo "set title font \"Time-Roman, 9\"">>gp
echo "set key font \"Time-Roman, 7\"">>gp
echo "set key at -55, 9">>gp

echo "set yrange [0:10]">>gp
echo "set xrange [0:99]">>gp
echo "set xtics font \"Time-Roman, 9\"">>gp
echo "set ytics font \"Time-Roman, 9\"">>gp
echo "set xlabel font \"Time-Roman, 9\"">>gp
echo "set ylabel font \"Time-Roman, 9\"">>gp
echo "set xlabel '% of interesting regions'" >>gp
echo "set ylabel '% of the environment that was missed'">>gp
#echo "set xtics (\"10%%\" 10, \"20%%\" 20, \"30%%\" 30, \"40%%\" 40, \"50%%\" 50, \"60%%\" 60, \"70%%\" 70, \"80%%\" 80, \"90%%\" 90, \"100%%\" 100)">>gp
#echo "set ytics ( \"1Km\" 1000, \"2km\" 2000, \"3km\" 3000, \"4km\" 4000, \"5km\" 5000, \"6km\" 6000, \"7km\" 7000, \"8km\"  8000, \"9km\"  9000, \"10km\" 10000)">>gp
echo "set ytics offset 0.7">> gp
echo "set tics nomirror">> gp
if [ $P -gt 1 ]
then
	echo "unset key">>gp
        echo "set title \"$P interesting patches\"" >>gp
	echo "unset ylabel">> gp
	#echo "unset ytics">> gp
	echo "set lmargin 2.8">> gp
	
else
	echo "set key left">>gp
        echo "set title \"$P interesting patch\"" >>gp
	#echo "set rmargin .8">> gp
	echo "set ylabel offset 2.7">>gp
fi

echo "f(x)=4180">>gp
#echo "set xlabel \"% of the environment that is interesting\" font \",8\"">>gp
#echo "set ylabel \"Length of the coverage path (m)\" font \",8\"">>gp
echo "set pointsize 0.7">>gp
echo "plot 'fn1' using 2:(\$1==$P?\$3*\$2:1/0) with lines ls 3 notitle, \\">>gp
echo       "'fn2' using 2:(\$1==$P?\$3*\$2:1/0) with lines ls 9 notitle, \\">>gp
echo       "'fn3' using 2:(\$1==$P?\$3*\$2:1/0) with lines ls 25 notitle, \\">>gp
echo       "'fn4' using 2:(\$1==$P?\$3*\$2:1/0) with lines ls 4 notitle, \\">>gp
echo       "'fn5' using 2:(\$1==$P?\$3*\$2:1/0) with lines ls 16 notitle, \\">>gp
echo       "'fn5' using 2:(\$1==$P?\$3*\$2:1/0):4 with yerrorbars ls 16 title 'C(0.2,0.3,0.2,0.3)', \\">>gp
echo       "'fn4' using 2:(\$1==$P?\$3*\$2:1/0):4 with yerrorbars ls 4 title 'C(0.01,0.001,0.1,0.4)', \\">>gp
echo       "'fn3' using 2:(\$1==$P?\$3*\$2:1/0):4 with yerrorbars ls 25 title 'C(0.1,0.4,0.01,0.001)', \\">>gp
echo       "'fn2' using 2:(\$1==$P?\$3*\$2:1/0):4 with yerrorbars ls 9 title 'C(0.1,0.4,0.1,0.4)', \\">>gp
echo       "'fn1' using 2:(\$1==$P?\$3*\$2:1/0):4 with yerrorbars ls 3 title 'C(0.001,0.01,0.001,0.01)' ">>gp
#echo 	   "'per' using 1:(\$2+\$3) with lines title 'min bound', \\">>gp
#echo 	   "f(x) with lines title 'Lawnmower'"	     >>gp
done
echo "unset multiplot">>gp

gnuplot gp
evince graph.pdf &
#cd ..      
       
       
       
