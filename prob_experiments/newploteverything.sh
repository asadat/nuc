echo "set terminal pdf size 12cm, 6cm" > gp
echo "set output 'graph.pdf'" >> gp
echo "set lmargin 4">> gp
echo "set multiplot layout 1,2" >>gp
for P in `seq 1 3 4`
do
echo "set title font \"Time-Roman, 7\"">>gp
echo "set key font \"Time-Roman, 6\"">>gp
echo "set key at 80, 7100">>gp

echo "set yrange [0:7199]">>gp
echo "set xrange [0:99]">>gp
echo "set xtics font \"Time-Roman, 6\"">>gp
echo "set ytics font \"Time-Roman, 6\"">>gp
echo "set xlabel font \"Time-Roman, 6\"">>gp
echo "set ylabel font \"Time-Roman, 6\"">>gp
echo "set xlabel '% of interesting regions'" >>gp
echo "set ylabel 'path length (m)'">>gp
echo "set xtics (\"10%%\" 10, \"20%%\" 20, \"30%%\" 30, \"40%%\" 40, \"50%%\" 50, \"60%%\" 60, \"70%%\" 70, \"80%%\" 80, \"90%%\" 90, \"100%%\" 100)">>gp
echo "set ytics ( \"1Km\" 1000, \"2km\" 2000, \"3km\" 3000, \"4km\" 4000, \"5km\" 5000, \"6km\" 6000, \"7km\" 7000, \"9km\"  9000, \"10km\" 10000)">>gp
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
	echo "set key right">>gp
	echo "set key horizontal">>gp
	echo "set key samplen 2 spacing 0.6">>gp
        echo "set title \"$P interesting patch\"" >>gp
	echo "set rmargin 1.0">> gp
	echo "set ylabel offset 2.5">>gp
fi

echo "f(x)=4180">>gp
#echo "set xlabel \"% of the environment that is interesting\" font \",8\"">>gp
#echo "set ylabel \"Length of the coverage path (m)\" font \",8\"">>gp
echo "set pointsize 0.9">>gp
echo "plot 'newc1' using 2:(\$1==$P?\$3:1/0) with lines ls 3 notitle, \\">>gp
echo       "'newc2' using 2:(\$1==$P?\$3:1/0) with lines ls 9 notitle, \\">>gp
echo       "'newc3' using 2:(\$1==$P?\$3:1/0) with lines ls 25 notitle, \\">>gp
echo       "'newc4' using 2:(\$1==$P?\$3:1/0) with lines ls 4 notitle, \\">>gp
echo       "'newc5' using 2:(\$1==$P?\$3:1/0) with lines ls 16 notitle, \\">>gp
echo       "'newc6' using 2:(\$1==$P?\$3:1/0) with lines ls 21 notitle, \\">>gp
echo       "'newc6' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 21 title 'C(0.2,0.4,0.2,0.4)', \\">>gp
echo       "'newc5' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 16 title 'C(0.15,0.4,0.15,0.4)', \\">>gp
echo       "'newc4' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 4 title 'C(0.01,0.001,0.1,0.4)', \\">>gp
echo       "'newc3' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 25 title 'C(0.1,0.4,0.001,0.01)', \\">>gp
echo       "'newc2' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 9 title 'C(0.1,0.4,0.1,0.4)', \\">>gp
echo       "'newc1' using 2:(\$1==$P?\$3:1/0):4 with yerrorbars ls 3 title 'C(0.001,0.01,0.001,0.001)', \\">>gp
#echo 	   "'per' using 1:(\$2+\$3) with lines title 'min bound', \\">>gp
echo 	   "f(x) with lines title 'Lawnmower'"	     >>gp
done
echo "unset multiplot">>gp

gnuplot gp
evince graph.pdf &
#cd ..      
       
       
       