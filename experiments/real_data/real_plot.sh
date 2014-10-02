echo "set terminal pdf size 10cm, 6cm" > gp
echo "set output 'real_graph.pdf'" >> gp
echo "set key right">>gp 
echo "set key vertical">>gp 
echo "set yrange [0:1]">>gp
echo "set xrange [300:699]">>gp
#echo "set title \"$P intresting patches\"" >>gp
echo "set key font \"Time-Roman, 4\"">>gp
echo "set key at 330, 0.7">>gp
echo "set key samplen 1" >> gp
echo "set key spacing 1" >> gp
echo "set key invert" >>gp
echo "set xtics font \"Time-Roman, 5\"">>gp
echo "set ytics font \"Time-Roman, 5\"">>gp
echo "set xlabel font \"Time-Roman, 5\"">>gp
echo "set ylabel font \"Time-Roman, 5\"">>gp
echo "set xlabel 'Travelled distance by the UAV'" >>gp
#echo "set ylabel 'path length (m)'">>gp
echo "set xtics (\"350m\" 350, \"400m\" 400, \"450m\" 450, \"500m\" 500, \"550m\" 550, \"600m\" 600, \"650m\" 650)">>gp
echo "set ytics ( \"25%%\" 0.2, \"50%%\" 0.5, \"75%%\" 0.75)">>gp
echo "set tics nomirror" >>gp
#echo "unset ytics" >> gp
echo "set border 3" >>gp
echo "set pointsize 1.8">>gp
#echo "set xlabel offset 9,0.3" >>gp
#echo "set label 'Our method' at 120,0.5 font \"Time-Roman, 5\"" >>gp
#echo "set label 'Lawnmower' at 120,0.22 font \"Time-Roman, 5\"" >>gp

#echo "set label 'Perceived' at 220,0.82 font \"Time-Roman, 4\"" >>gp
#echo "set label 'Interestingess' at 220,0.75 font \"Time-Roman, 4\"" >>gp
echo "set ylabel '% of area perceived as interesting'">>gp 
echo "unset key" >> gp
echo "plot 'realdata' using 1:(\$3==10?1:1/0):2 with xerrorbars ls 13 title ' ' , \\">>gp
echo       "'realdata' using 1:(\$3==1?\$2:1/0) ls 11 notitle ">> gp

gnuplot gp
evince real_graph.pdf &
cd ..      
       
       
       
