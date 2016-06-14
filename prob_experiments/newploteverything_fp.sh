echo "set terminal pdf size 12cm, 7cm" > gp
echo "set output 'graph.pdf'" >> gp
echo "set lmargin 4">> gp
echo "set multiplot layout 1,2" >>gp
for P in `seq 1 3 4`
do
echo "set title font \"Time-Roman, 9\"">>gp
echo "set key font \"Time-Roman, 7\"">>gp
echo "set key at -10, 50">>gp

echo "set yrange [0:50]">>gp
echo "set xrange [0:99]">>gp
echo "set xtics font \"Time-Roman, 8\"">>gp
echo "set ytics font \"Time-Roman, 8\"">>gp
echo "set xlabel font \"Time-Roman, 9\"">>gp
echo "set ylabel font \"Time-Roman, 9\"">>gp
echo "set xlabel '% of interesting regions'" >>gp
echo "set ylabel '% of the environment that was wrongly covered'">>gp
echo "set xtics (\"10%%\" 10, \"20%%\" 20, \"30%%\" 30, \"40%%\" 40, \"50%%\" 50, \"60%%\" 60, \"70%%\" 70, \"80%%\" 80, \"90%%\" 90, \"100%%\" 100)">>gp
echo "set ytics (\"5%%\" 5, \"10%%\" 10, \"15%%\" 15, \"20%%\" 20, \"25%%\" 25, \"30%%\" 30, \"35%%\" 35)" >> gp
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
	echo "set rmargin .8">> gp
	echo "set ylabel offset 2.4">>gp
fi

echo "f(x)=4180">>gp
#echo "set xlabel \"% of the environment that is interesting\" font \",8\"">>gp
#echo "set ylabel \"Length of the coverage path (m)\" font \",8\"">>gp
echo "set pointsize 0.7">>gp
echo "plot 'newfp1' using 2:(\$1==$P?\$3*100:1/0) with lines ls 3 notitle, \\">>gp
echo       "'newfp2' using 2:(\$1==$P?\$3*100:1/0) with lines ls 9 notitle, \\">>gp
echo       "'newfp3' using 2:(\$1==$P?\$3*100:1/0) with lines ls 25 notitle, \\">>gp
echo       "'newfp4' using 2:(\$1==$P?\$3*100:1/0) with lines ls 4 notitle, \\">>gp
echo       "'newfp5' using 2:(\$1==$P?\$3*100:1/0) with lines ls 16 notitle, \\">>gp
echo       "'newfp6' using 2:(\$1==$P?\$3*100:1/0) with lines ls 21 notitle, \\">>gp
echo       "'newfp6' using 2:(\$1==$P?\$3*100:1/0):(\$4*100) with yerrorbars ls 21 title 'C(0.2,0.4,0.2,0.4)', \\">>gp
echo       "'newfp5' using 2:(\$1==$P?\$3*100:1/0):(\$4*100) with yerrorbars ls 16 title 'C(0.15,0.4,0.15,0.4)', \\">>gp
echo       "'newfp4' using 2:(\$1==$P?\$3*100:1/0):(\$4*100) with yerrorbars ls 4 title 'C(0.01,0.001,0.1,0.4)', \\">>gp
echo       "'newfp3' using 2:(\$1==$P?\$3*100:1/0):(\$4*100) with yerrorbars ls 25 title 'C(0.1,0.4,0.01,0.001)', \\">>gp
echo       "'newfp2' using 2:(\$1==$P?\$3*100:1/0):(\$4*100) with yerrorbars ls 9 title 'C(0.1,0.4,0.1,0.4)', \\">>gp
echo       "'newfp1' using 2:(\$1==$P?\$3*100:1/0):(\$4*100) with yerrorbars ls 3 title 'C(0.001,0.01,0.001,0.01)' ">>gp
#echo "plot 'fp1' using 2:(\$1==$P?\$3*100:1/0) with lines ls 3 notitle, \\">>gp
#echo       "'fp2' using 2:(\$1==$P?\$3*100:1/0) with lines ls 9 notitle, \\">>gp
#echo       "'fp3' using 2:(\$1==$P?\$3*100:1/0) with lines ls 25 notitle, \\">>gp
#echo       "'fp4' using 2:(\$1==$P?\$3*100:1/0) with lines ls 23 notitle, \\">>gp
#echo       "'fp5' using 2:(\$1==$P?\$3*100:1/0) with lines ls 28 notitle, \\">>gp
#echo       "'fp5' using 2:(\$1==$P?\$3*100:1/0):4 with yerrorbars ls 8 title 'c5', \\">>gp
#echo       "'fp4' using 2:(\$1==$P?\$3*100:1/0):4 with yerrorbars ls 5 title 'c4', \\">>gp
#echo       "'fp3' using 2:(\$1==$P?\$3*100:1/0):4 with yerrorbars ls 3 title 'c3', \\">>gp
#echo       "'fp2' using 2:(\$1==$P?\$3*100:1/0):4 with yerrorbars ls 9 title 'c2', \\">>gp
#echo       "'fp1' using 2:(\$1==$P?\$3*100:1/0):4 with yerrorbars ls 25 title 'Hilbert c1' ">>gp
#echo 	   "'per' using 1:(\$2+\$3) with lines title 'min bound', \\">>gp
#echo 	   "f(x) with lines title 'Lawnmower'"	     >>gp
done
echo "unset multiplot">>gp

gnuplot gp
evince graph.pdf &
#cd ..      
       
       
       
