fn=$(echo $1 | sed "s/\..*//g")
mkdir $fn
mv $1 "$fn/."
mv *.jpg "$fn/."

