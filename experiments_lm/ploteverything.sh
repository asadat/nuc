cd tables
echo "set terminal pdf size 25cm, 20cm" > gp
echo "set output 'graph.pdf'" >> gp
echo "set lmargin 4">> gp
echo "set multiplot layout 2,3" >>gp


for OUTPUT in `seq 1 1 2`
do

for INTR in `seq 20 20 60`
do

echo "set title font \"Time-Roman, 7\"">>gp
echo "set key font \"Time-Roman, 7\"">>gp
#echo "set key at -10, 9000">>gp

echo "set xrange [0:21]">>gp
echo "set xtics font \"Time-Roman, 5\"">>gp
echo "set ytics font \"Time-Roman, 5\"">>gp
echo "set xlabel font \"Time-Roman, 5\"">>gp
echo "set ylabel font \"Time-Roman, 5\"">>gp
echo "set xlabel '% patches'" >>gp

if [ $OUTPUT -eq 1 ] 
then
echo "set ylabel 'path length (m)'">>gp
echo "set yrange [0:4900]">>gp
echo "set ytics ( \"1Km\" 1000, \"2km\" 2000, \"3km\" 3000, \"4km\" 4000, \"5km\" 5000, \"6km\" 6000, \"7km\" 7000, \"8km\"  8000, \"9km\"  9000, \"10km\" 10000)">>gp
echo "f(x)=4180">>gp

else

echo "set yrange [0:800]">>gp
echo "set ytics 50">>gp
echo "set ylabel '#Turns'">>gp
echo "f(x)=63">>gp

fi

echo "set ytics offset 0.7">> gp
echo "set tics nomirror">> gp
if [ $INTR -gt 20 ]
then
	echo "unset key">>gp
        echo "set title \"$INTR% interesting\"" >>gp
	echo "unset ylabel">> gp
	echo "set lmargin 3.5">> gp
	
else
	if [ $OUTPUT -eq 1 ] 
	then
	        echo "set ylabel \"Path length (m)\" font \",7\" offset 1.5">>gp
		echo "set key right bottom">>gp
	else
	        echo "set ylabel \"#Turns\" font \",7\" offset 1.5">>gp
		echo "set key right top">>gp
	fi

	echo "set title \"$INTR% interesting\"" >>gp
	echo "set lmargin -2">> gp
	echo "set rmargin 0.5">> gp
fi




#echo "set xlabel \"% of the environment that is interesting\" font \",8\"">>gp


if [ $OUTPUT -eq 1 ] 
then
echo "set pointsize 1.5">>gp
echo "plot  'hi-0' using 1:(\$2==$INTR?\$3:1/0) with lines ls 25 notitle, \\">>gp
echo       "'hi-0' using 1:(\$2==$INTR?\$3:1/0):4 with yerrorbars ls 25 title 'LML:0', \\">>gp
echo 	   "'hi-1' using 1:(\$2==$INTR?\$3:1/0) with lines ls 26 notitle, \\">>gp
echo       "'hi-1' using 1:(\$2==$INTR?\$3:1/0):4 with yerrorbars ls 26 title 'LML:1', \\">>gp
echo 	   "'hi-2' using 1:(\$2==$INTR?\$3:1/0) with lines ls 29 notitle, \\">>gp
echo       "'hi-2' using 1:(\$2==$INTR?\$3:1/0):4 with yerrorbars ls 29 title 'LML:2', \\">>gp
echo	   "'hi-3' using 1:(\$2==$INTR?\$3:1/0) with lines ls 28 notitle, \\">>gp
echo       "'hi-3' using 1:(\$2==$INTR?\$3:1/0):4 with yerrorbars ls 28 title 'LML:3', \\">>gp
echo 	   "f(x) with lines title 'Lawnmower'"	     >>gp

else

echo "set pointsize 1.5">>gp
echo "plot  'hi-0' using 1:(\$2==$INTR?\$9:1/0) with lines ls 25 notitle, \\">>gp
echo       "'hi-0' using 1:(\$2==$INTR?\$9:1/0):10 with yerrorbars ls 25 title 'LML:0', \\">>gp
echo 	   "'hi-1' using 1:(\$2==$INTR?\$9:1/0) with lines ls 26 notitle, \\">>gp
echo       "'hi-1' using 1:(\$2==$INTR?\$9:1/0):10 with yerrorbars ls 26 title 'LML:1', \\">>gp
echo 	   "'hi-2' using 1:(\$2==$INTR?\$9:1/0) with lines ls 29 notitle, \\">>gp
echo       "'hi-2' using 1:(\$2==$INTR?\$9:1/0):10 with yerrorbars ls 29 title 'LML:2', \\">>gp
echo	   "'hi-3' using 1:(\$2==$INTR?\$9:1/0) with lines ls 28 notitle, \\">>gp
echo       "'hi-3' using 1:(\$2==$INTR?\$9:1/0):10 with yerrorbars ls 28 title 'LML:3', \\">>gp
echo 	   "f(x) with lines title 'Lawnmower'"	     >>gp

fi

done
done
echo "unset multiplot">>gp

gnuplot gp
evince graph.pdf &
cd ..      
       
       
       
