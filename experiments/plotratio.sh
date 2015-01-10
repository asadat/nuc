cd tables
echo "set terminal pdf size 8cm, 5cm" > rgp
echo "set output 'ratio_graph.pdf'" >> rgp
echo "set key right">>rgp
echo "set key vertical">>rgp
echo "set yrange [0:0.35]">>rgp
echo "set xrange [0:99]">>rgp
#echo "set title \"$P intresting patches\"" >>rgp
echo "set key font \"Time-Roman, 4\"">>rgp
#echo "set key at 330, 0.7">>rgp
#echo "set key samplen 1" >> rgp
#echo "set key spacing 1" >> rgp
#echo "set key invert" >>rgp
echo "set key box">>rgp
echo "set xtics font \"Time-Roman, 4\"">>rgp
echo "set ytics font \"Time-Roman, 4\"">>rgp
echo "set xlabel font \"Time-Roman, 4\"">>rgp
echo "set ylabel font \"Time-Roman, 4\"">>rgp
echo "set xlabel 'Percentage of the area that was interesting'" >>rgp
echo "set ylabel 'Displacement along z-axis to total trajectory length'">>rgp
echo "set xtics (\"10%%\" 10, \"20%%\" 20, \"30%%\" 30, \"40%%\" 40, \"50%%\" 50, \"60%%\" 60, \"70%%\" 70, \"80%%\" 80, \"90%%\" 90, \"100%%\" 100)">>rgp
#echo "set ytics ( \"25%%\" 0.2, \"50%%\" 0.5, \"75%%\" 0.75)">>rgp
echo "set tics nomirror" >>rgp
#echo "unset ytics" >> rgp
echo "set border 3" >>rgp
echo "set pointsize 1">>rgp
#echo "set xlabel offset 9,0.3" >>rgp
#echo "set label 'Our method' at 120,0.5 font \"Time-Roman, 5\"" >>rgp
#echo "set label 'Lawnmower' at 120,0.22 font \"Time-Roman, 5\"" >>rgp

#echo "set label 'Perceived' at 220,0.82 font \"Time-Roman, 4\"" >>rgp
#echo "set label 'Interestingess' at 220,0.75 font \"Time-Roman, 4\"" >>rgp
#echo "set ylabel '% of area perceived as interesting'">>rgp
#echo "unset key" >> rgp
echo "plot 'hi' using 2:(\$1==1?\$5/\$3:1/0) ls 3 with linespoints title '1 patch' , \\">>rgp
echo       "'hi' using 2:(\$1==2?\$5/\$3:1/0) ls 9 with linespoints title '2 patches', \\">> rgp
echo       "'hi' using 2:(\$1==3?\$5/\$3:1/0) ls 25 with linespoints title '3 patches', \\">> rgp
echo       "'hi' using 2:(\$1==4?\$5/\$3:1/0) ls 16 lc 9 with linespoints title '4 patches'">> rgp

gnuplot rgp
evince ratio_graph.pdf &
cd ..      
       
       
       
