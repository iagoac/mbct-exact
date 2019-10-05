#!/bin/bash

echo "size,degree,rep,exact,npe,intersetion_size,intersection_percent,m2,m3" > results.csv
for i in exact/*.dat; 
  do ./evaluate $i >> results.csv; 
done

sed -i 's/exact\/a-//g' results.csv;
sed -i 's/.dat//g' results.csv;
sed -i 's/-/,/g' results.csv;
