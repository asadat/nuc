for CONG in `seq 1 1 6`
do
./result-table.sh "newresults-$CONG" hi > "newc$CONG"
./result-table-fp.sh "newresults-$CONG" hi > "newfp$CONG"
./result-table-fn.sh "newresults-$CONG" hi > "newfn$CONG"
done
